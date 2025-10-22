import React, { useState, useRef } from "react";
import { useNavigate } from "react-router-dom";
import TranscriptViewer from "../InitialPromptDisplay";

const InitalPromptAssistant: React.FC = () => {
  // Record landing time in localStorage
  React.useEffect(() => {
    localStorage.setItem('initialPromptAssistant_landingTime', new Date().toISOString());
  }, []);
  
  const navigate = useNavigate();
  const [clearTranscriptTrigger, setClearTranscriptTrigger] = useState(0);
  const [startCount, setStartCount] = useState(0);
  const transcriptViewerRef = useRef<any>(null);

  const [progress, setProgress] = useState({
    conversationStarted: false,
    conversationEnded: false,
  });

  const handleStart = async () => {
    try {
      // Save prompt if not the first time
      if (startCount > 0 && transcriptViewerRef.current && transcriptViewerRef.current.savePrompt) {
        await transcriptViewerRef.current.savePrompt();
      }
      const participantId = localStorage.getItem("participantId") || "";
      await fetch(`http://localhost:8000/start-initial-prompt-conversation?participantId=${encodeURIComponent(participantId)}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify("start conversation"),
      });
      setProgress((prev) => ({
        ...prev,
        conversationStarted: true,
        conversationEnded: false,
      }));
      setClearTranscriptTrigger((prev) => prev + 1);
      setStartCount((prev) => prev + 1);
    } catch (error) {
      alert("Failed to start conversation.");
      console.error("Error starting conversation:", error);
    }
  };

  // Handle "End Conversation"
  const handleEnd = async () => {
    // Save the initial prompt before navigating
    if (transcriptViewerRef.current && transcriptViewerRef.current.savePrompt) {
      try {
        await transcriptViewerRef.current.savePrompt();
      } catch (err) {
        alert("Failed to save the initial prompt before continuing.");
        console.error("Error saving initial prompt:", err);
      }
    }

    try {
      await fetch("http://localhost:8000/end-conversation", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify("end conversation"),
      });
      setProgress((prev) => ({
        ...prev,
        conversationStarted: false,
        conversationEnded: true,
      }));
    } catch (error) {
      alert("Failed to end conversation.");
      console.error("Error ending conversation:", error);
    }
  };

  // Handle "Done"
  const handleDone = async (e: { preventDefault: () => void }) => {
    e.preventDefault();

    const confirmed = window.confirm("Are you sure you want to move on?");
    if (!confirmed) return;

    const now = new Date().toISOString();
    localStorage.setItem("initialPromptAssistant_EndTime", now);
    const startTime = localStorage.getItem("initialPromptAssistant_landingTime");

    const timeTaken = startTime
      ? ((new Date(now).getTime() - new Date(startTime).getTime()) / 1000).toString()
      : "0";

    localStorage.setItem("initialPromptAssistant_TimeTaken", timeTaken);
    
    // Save the initial prompt before navigating
    if (transcriptViewerRef.current && transcriptViewerRef.current.savePrompt) {
      try {
        await transcriptViewerRef.current.savePrompt();
      } catch (err) {
        alert("Failed to save the initial prompt before continuing.");
        console.error("Error saving initial prompt:", err);
      }
    }
    
    try {
      await fetch("http://localhost:8000/rename-transcript", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify("rename transcript"),
      });
    } catch (error) {
      alert("Failed to rename transcript.");
      console.error("Error renaming transcript:", error);
    }

    // call end_conversation if the user didn't press "End Testing" before pressing "Done"
    if (progress.conversationStarted && !progress.conversationEnded) {
      try {
        await fetch("http://localhost:8000/end-conversation", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify("end conversation"),
        });
      } catch (error) {
        alert("Failed to end conversation.");
        console.error("Error publishing prompt:", error);
      }
    }

    navigate("/experimental");
  };

  const getInstruction = () => {
    if (!progress.conversationStarted) {
      return 'Press "Start Conversation" to begin your conversation with Ace to craft an initial prompt for your task together. As you talk with Ace, Ace will help you draft the prompt automatically. You can edit the prompt by typing in the textbox at any time during the conversation. Click "Save" to tell Ace to incorporate the edited prompt. When you are satisfied with the prompt, click "End Conversation".';
    }
    if (progress.conversationStarted && !progress.conversationEnded) {
      return 'You can edit the current prompt and click "Save" to save your changes for Ace to incorporate. When you are satisfied with the prompt, click "End Conversation".';
    }
    return 'Initial prompt creation is complete. Click "Next" to continue to the main study.';
  };

  return (
    <div className="w-full h-full">
      <div className="p-4 border rounded bg-blue-100 text-sm mb-5 max-w-5xl mx-auto">
        <strong>Instruction:</strong> {getInstruction()}
      </div>
      <TranscriptViewer ref={transcriptViewerRef} progress={progress} clearTranscriptTrigger={clearTranscriptTrigger} />
      <div className="p-5 flex gap-3 justify-center">
        <button
          className={`px-5 py-3 text-sm bg-green-500 hover:bg-green-600 text-white rounded ${(progress.conversationStarted) ? 'opacity-50 cursor-not-allowed' : ''}`}
          onClick={handleStart}
          disabled={progress.conversationStarted}
          type="button"
        >
          Start Conversation
        </button>
        <button
          className={`px-5 py-3 text-sm bg-red-500 hover:bg-red-600 text-white rounded ${(!progress.conversationStarted || progress.conversationEnded) ? 'opacity-50 cursor-not-allowed' : ''}`}
          onClick={handleEnd}
          disabled={!progress.conversationStarted || progress.conversationEnded}
          type="button"
        >
          End Conversation
        </button>
      </div>
      <button
        className={`px-5 py-3 text-sm bg-blue-500 hover:bg-blue-600 text-white rounded ${(!progress.conversationEnded) ? 'opacity-50 cursor-not-allowed' : ''}`}
        onClick={handleDone}
        disabled={!progress.conversationEnded}
        type="button"
      >
        Next
      </button>
    </div>
  );
};

export default InitalPromptAssistant;