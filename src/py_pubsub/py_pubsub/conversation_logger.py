import datetime
import os
from pathlib import Path
from typing import Dict, Any, Optional
from enum import Enum
import threading
import queue
import json

class EventType(Enum):
    """Enumeration of conversation event types."""
    USER_SPEECH = "USER_SPEECH"
    USER_SHORT_SPEECH = "USER_SHORT_SPEECH"
    USER_INTERRUPTION = "USER_INTERRUPTION"
    ROBOT_SPEECH = "ROBOT_SPEECH"
    ROBOT_SPEECH_REMAINING = "ROBOT_SPEECH_REMAINING"
    WAKEWORD_DETECTED = "WAKEWORD_DETECTED"
    SYSTEM_EVENT = "SYSTEM_EVENT"  # Added missing system event type

class ConversationLogger:
    """Thread-safe logger for conversation events with structured format."""
    
    def __init__(self, log_directory: str = "/home/icl/2026-hri-conversation-design-experimental/txt_files/", 
                 session_id: Optional[str] = None):
        self.log_directory = Path(log_directory + "logs")
        self.log_directory.mkdir(parents=True, exist_ok=True)
        self.transcript_directory = Path(log_directory + "transcripts")
        self.transcript_directory.mkdir(parents=True, exist_ok=True)

        # Generate session ID if not provided
        if session_id is None:
            session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_id = session_id
        
        # Create log files - fixed extension consistency
        self.conversation_log = self.log_directory / f"conversation_latest.jsonl"
        self.summary_log = self.log_directory / f"summary_latest.txt"
        self.transcript_log = self.transcript_directory / f"transcript_latest.txt"
        
        # Thread-safe logging
        self.log_queue = queue.Queue()
        self.stop_logging = False
        self.log_thread = threading.Thread(target=self._log_worker, daemon=True)
        self.log_thread.start()
        
        # Session metadata
        self.session_start_time = datetime.datetime.now()
        self.event_counter = 0
        
        # Log session start
        self.log_system_event("SESSION_START", {"session_id": session_id})
    
    def _get_timestamp(self) -> str:
        """Get current timestamp in ISO format."""
        return datetime.datetime.now().isoformat()
    
    def _log_worker(self):
        """Background worker for thread-safe logging."""
        while not self.stop_logging:
            try:
                log_entry = self.log_queue.get(timeout=1.0)
                if log_entry is None:
                    break
                
                # Write to JSONL file (structured)
                with open(self.conversation_log, 'a', encoding='utf-8') as f:
                    f.write(json.dumps(log_entry, ensure_ascii=False) + '\n')
                    f.flush()
                    os.fsync(f.fileno())
                
                # Write to human-readable summary
                self._write_summary_entry(log_entry)
                self._write_transcript_entry(log_entry)
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Logging error: {e}")
    
    def _write_summary_entry(self, entry: Dict[str, Any]):
        """Write human-readable summary entry."""
        timestamp = entry['timestamp']
        event_type = entry['event_type']
        
        # Format based on event type
        if event_type == EventType.USER_SPEECH.value:
            content = entry['content']['text']
            summary_line = f"[{timestamp}] USER: {content}"
        elif event_type == EventType.USER_SHORT_SPEECH.value:
            content = entry['content']['text']
            summary_line = f"[{timestamp}] USER_SHORT_SPEECH: {content}"
        elif event_type == EventType.USER_INTERRUPTION.value:
            interruption_type = entry['content'].get('interruption_type', 'unknown')
            text = entry['content'].get('text', '')
            summary_line = f"[{timestamp}] USER_INTERRUPTION ({interruption_type}): {text}"
        elif event_type == EventType.ROBOT_SPEECH.value:
            content = entry['content']['text']
            summary_line = f"[{timestamp}] ROBOT: {content}"
        elif event_type == EventType.ROBOT_SPEECH_REMAINING.value:
            remaining_text = entry['content']['text']
            summary_line = f"[{timestamp}] ROBOT_SPEECH_REMAINING: {remaining_text}"
        elif event_type == EventType.WAKEWORD_DETECTED.value:
            wakeword = entry['content'].get('wakeword', 'unknown')
            text = entry['content'].get('text', '')
            summary_line = f"[{timestamp}] WAKEWORD ({wakeword}): {text}"
        elif event_type == EventType.SYSTEM_EVENT.value:
            event_name = entry['content'].get('event_name', 'unknown')
            summary_line = f"[{timestamp}] SYSTEM: {event_name}"
        else:
            summary_line = f"[{timestamp}] {event_type}: {entry['content']}"
        
        try:
            with open(self.summary_log, 'a', encoding='utf-8') as f:
                f.write(summary_line + '\n')
                f.flush()
                os.fsync(f.fileno())
        except Exception as e:
            print(f"Error writing summary: {e}")


    def _write_transcript_entry(self, entry: Dict[str, Any]):
        """Write human-readable transcript entry."""
        event_type = entry['event_type']
        
        # Format based on event type
        if event_type == EventType.USER_SPEECH.value:
            content = entry['content']['text']
            transcript_line = f"USER: {content}"
        elif event_type == EventType.USER_INTERRUPTION.value:
            text = entry['content'].get('text', '')
            transcript_line = f"USER: {text}"
        elif event_type == EventType.USER_SHORT_SPEECH.value:
            text = entry['content'].get('text', '')
            transcript_line = f"USER: {text}"
        elif event_type == EventType.ROBOT_SPEECH.value:
            content = entry['content']['text']
            transcript_line = f"ROBOT: {content}"
        else:
            return
        
        try:
            with open(self.transcript_log, 'a', encoding='utf-8') as f:
                f.write(transcript_line + '\n')
                f.flush()
                os.fsync(f.fileno())
        except Exception as e:
            print(f"Error writing transcript: {e}")

    
    def _log_event(self, event_type: EventType, content: Dict[str, Any], 
                   metadata: Optional[Dict[str, Any]] = None):
        """Internal method to log an event."""
        self.event_counter += 1
        
        log_entry = {
            "session_id": self.session_id,
            "event_id": self.event_counter,
            "timestamp": self._get_timestamp(),
            "event_type": event_type.value,
            "content": content,
            "metadata": metadata or {}
        }
        
        self.log_queue.put(log_entry)
    
    # Public logging methods
    def log_user_speech(self, text: str, node_name: str = None):
        """Log user speech event."""
        content = {
            "text": text
        }
        metadata = {"source_node": node_name} if node_name else {}
        self._log_event(EventType.USER_SPEECH, content, metadata)
    
    def log_user_short_speech(self, text: str, node_name: str = None):
        """Log user speech event."""
        content = {
            "text": text
        }
        metadata = {"source_node": node_name} if node_name else {}
        self._log_event(EventType.USER_SHORT_SPEECH, content, metadata)

    def log_user_interruption(self, text: str, interruption_type: str, 
                             is_wakeword: bool = False, node_name: str = None):
        """Log user interruption event."""
        content = {
            "text": text,
            "interruption_type": interruption_type,
            "is_wakeword": is_wakeword
        }
        metadata = {"source_node": node_name} if node_name else {}
        self._log_event(EventType.USER_INTERRUPTION, content, metadata)
    
    def log_robot_speech(self, text: str, node_name: str = None):
        """Log robot speech event."""
        content = {
            "text": text
        }
        metadata = {"source_node": node_name} if node_name else {}
        self._log_event(EventType.ROBOT_SPEECH, content, metadata)

    def log_remaining_robot_speech(self, remaining_text: str, node_name: str = None):
        """Log robot speech interruption event."""
        content = {"text": remaining_text}
        metadata = {"source_node": node_name} if node_name else {}
        self._log_event(EventType.ROBOT_SPEECH_REMAINING, content, metadata)
    
    def log_wakeword_detected(self, wakeword: str, text: str, node_name: str = None):
        """Log wakeword detection event."""
        content = {
            "wakeword": wakeword,
            "text": text
        }
        metadata = {"source_node": node_name} if node_name else {}
        self._log_event(EventType.WAKEWORD_DETECTED, content, metadata)
    
    def log_system_event(self, event_name: str, data: Dict[str, Any], node_name: str = None):
        """Log system events."""
        content = {
            "event_name": event_name,
            "data": data
        }
        metadata = {"source_node": node_name} if node_name else {}
        self._log_event(EventType.SYSTEM_EVENT, content, metadata)
     
    def get_session_info(self) -> Dict[str, Any]:
        """Get current session information."""
        return {
            "session_id": self.session_id,
            "start_time": self.session_start_time.isoformat(),
            "total_events": self.event_counter,
            "log_files": {
                "conversation_log": str(self.conversation_log),
                "summary_log": str(self.summary_log)
            }
        }
    
    def close(self):
        """Close the logger and flush all logs."""
        self.log_system_event("SESSION_END", {
            "session_duration": str(datetime.datetime.now() - self.session_start_time),
            "total_events": self.event_counter
        })
        
        self.stop_logging = True
        self.log_queue.put(None)  # Signal to stop
        self.log_thread.join(timeout=5.0)
        
        print(f"Conversation log saved: {self.conversation_log}")
        print(f"Summary log saved: {self.summary_log}")
    
    def save_conversation_history(self, conversation_history, node_name: str = None):
        """Save the full conversation history to a file with a timestamp in the filename."""
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"conversation_history_{timestamp}.json"
        filepath = os.path.join(str(self.log_directory), filename)
        data = {
            "timestamp": timestamp,
            "node_name": node_name,
            "history": conversation_history
        }
        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"Error saving conversation history: {e}")


# Singleton instance for easy access across nodes
_conversation_logger = None
_logger_lock = threading.Lock()


def get_conversation_logger(log_directory: str = "/home/icl/2026-hri-conversation-design-experimental/txt_files/",
                          session_id: Optional[str] = None) -> ConversationLogger:
    """Get or create the global conversation logger instance."""
    global _conversation_logger
    
    with _logger_lock:
        if _conversation_logger is None:
            _conversation_logger = ConversationLogger(log_directory, session_id)
        return _conversation_logger


def close_conversation_logger():
    """Close the global conversation logger."""
    global _conversation_logger
    
    with _logger_lock:
        if _conversation_logger is not None:
            _conversation_logger.close()
            _conversation_logger = None