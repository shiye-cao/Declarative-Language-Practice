"""
Install required packages:
pip install fastapi uvicorn pydantic

Run in mac:
uvicorn main:app --reload --port 8000

Run in ros:
USE_ROS=true uvicorn main:app --reload --port 8000

CURL format:
curl -X POST http://localhost:8000/update-prompts \
  -H "Content-Type: application/json" \
  -d '{"prompts": {"baseline": "This is a test."}}'

"""
import zmq
import time
import datetime
import os
import json
from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from datetime import datetime
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse

USE_ROS = os.getenv("USE_ROS", "false").lower() == "true"

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Or ["http://localhost:3000"] for tighter security
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

STATIC_TXT_DIR = os.path.join(BASE_DIR, "../txt_files")
os.makedirs(STATIC_TXT_DIR, exist_ok=True)

TRANSCRIPT_FOLDER = os.path.join(BASE_DIR, "../txt_files/transcripts")
os.makedirs(TRANSCRIPT_FOLDER, exist_ok=True)

SAVE_FOLDER = os.path.join(BASE_DIR, "../txt_files/analysis_results")
os.makedirs(SAVE_FOLDER, exist_ok=True)

PROMPT_FOLDER = os.path.join(BASE_DIR, "../py")
app.mount("/txt_files", StaticFiles(directory=STATIC_TXT_DIR), name="txt_files")

LOG_FOLDER = os.path.join(BASE_DIR, "../txt_files/logs")
os.makedirs(LOG_FOLDER, exist_ok=True)

HISTORY_FOLDER = os.path.join(BASE_DIR, "../txt_files/history")
os.makedirs(HISTORY_FOLDER, exist_ok=True)

INITIAL_PROMPT_FOLDER = os.path.join(BASE_DIR, "../txt_files/initial_prompts")
os.makedirs(INITIAL_PROMPT_FOLDER, exist_ok=True)

import os
import shutil
from datetime import datetime
from zoneinfo import ZoneInfo
from typing import Dict, Any, List
from fastapi import Body


SUBFOLDERS = ["analysis_results", "history", "initial_prompts", "logs", "transcripts"]
ARCHIVE_ROOT = os.path.join(STATIC_TXT_DIR, "archived")
os.makedirs(ARCHIVE_ROOT, exist_ok=True)

def _unique_archive_batch_dir(pid: str, date_str: str) -> str:
    """
    Returns a unique directory path under ARCHIVE_ROOT like:
    archived/PID_YYYY-MM-DD[_n]
    """
    base = f"{pid}_{date_str}"
    batch_dir = os.path.join(ARCHIVE_ROOT, base)
    n = 1
    while os.path.exists(batch_dir):
        batch_dir = os.path.join(ARCHIVE_ROOT, f"{base}_{n}")
        n += 1
    return batch_dir

def _ensure_empty_dir(path: str):
    os.makedirs(path, exist_ok=True)
    # optional: drop a .gitkeep so empty dirs persist in git
    gk = os.path.join(path, ".gitkeep")
    try:
        open(gk, "a").close()
    except Exception:
        pass

@app.post("/archive-study")
def archive_study(payload: Dict[str, Any] = Body(...)):
    """
    Body: { "pid": "P001" }
    Moves all text subfolders into ../txt_files/archived/PID_YYYY-MM-DD[_n]/,
    then recreates empty folders for the next study.
    """
    pid = (payload.get("pid") or "").strip()
    if not pid:
        return {"ok": False, "error": "Missing or invalid 'pid'."}

    # Use your preferred timezone for dating the archive
    date_str = datetime.now(ZoneInfo("Asia/Seoul")).date().isoformat()

    # Make a unique batch folder for this archive
    batch_dir = _unique_archive_batch_dir(pid, date_str)
    os.makedirs(batch_dir, exist_ok=True)

    moved: List[Dict[str, str]] = []
    errors: List[Dict[str, str]] = []

    # 1) Move subfolders into the archive batch
    for name in SUBFOLDERS:
        src = os.path.join(STATIC_TXT_DIR, name)
        dest = os.path.join(batch_dir, name)
        try:
            if os.path.isdir(src):
                # If dest's parent exists, shutil.move will move/merge accordingly
                os.makedirs(os.path.dirname(dest), exist_ok=True)
                shutil.move(src, dest)
                moved.append({"from": src, "to": dest})
            else:
                # Not present this run; note and continue
                errors.append({"folder": src, "error": "Source folder does not exist"})
        except Exception as e:
            errors.append({"folder": src, "error": str(e)})

    # 2) Recreate clean, empty subfolders for the next study
    recreated: List[str] = []
    for name in SUBFOLDERS:
        fresh = os.path.join(STATIC_TXT_DIR, name)
        try:
            _ensure_empty_dir(fresh)
            recreated.append(fresh)
        except Exception as e:
            errors.append({"folder": fresh, "error": str(e)})

    return {
        "ok": True,
        "pid": pid,
        "date": date_str,
        "archive_batch": batch_dir,
        "moved_count": len(moved),
        "moved": moved,
        "recreated": recreated,
        "errors": errors,
    }


class PromptPayload(BaseModel):
    prompts: dict  # Should include "baseline", etc.

class SaveResultPayload(BaseModel):
    filename: str
    content: str

@app.post("/save-analysis")
def save_analysis(payload: SaveResultPayload):
    try:
        # Optional: timestamped to avoid overwrites
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Ensure it's a clean filename
        safe_name = os.path.basename(payload.filename)
        if not safe_name.endswith(".txt"):
            safe_name += timestamp + ".txt"

        full_path = os.path.join(SAVE_FOLDER, f"{safe_name}")

        with open(full_path, "w", encoding="utf-8") as f:
            f.write(payload.content)

        return {"status": "success", "saved_to": full_path}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/save-data")
def save_data(payload: SaveResultPayload):
    try:
        # Ensure it's a clean filename
        safe_name = os.path.basename(payload.filename)
        if not safe_name.endswith(".txt"):
            safe_name += ".txt"

        full_path = os.path.join(ARCHIVE_ROOT, f"{safe_name}")

        with open(full_path, "w", encoding="utf-8") as f:
            f.write(payload.content)

        return {"status": "success", "saved_to": full_path}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/latest-transcript", response_class=PlainTextResponse)
def latest_transcript():
    try:
        files_name = "transcript_latest.txt"
        # if not files:
        #     raise HTTPException(status_code=404, detail="No transcript files found.")
        # # Sort by modified time, newest first
        # files.sort(key=lambda f: os.path.getmtime(os.path.join(TRANSCRIPT_FOLDER, f)), reverse=True)
        latest_file = os.path.join(TRANSCRIPT_FOLDER, files_name)
        with open(latest_file, "r", encoding="utf-8") as f:
            return f.read()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/latest-transcript/stream")
async def latest_transcript_stream():
    async def event_generator():
        last_mtime = 0.0
        while True:
            try:
                files_name = "transcript_latest.txt"
                if files_name in os.listdir(TRANSCRIPT_FOLDER):
                    path = os.path.join(TRANSCRIPT_FOLDER, files_name)
                    mtime = os.path.getmtime(path)
                    if mtime != last_mtime:
                        with open(path, "r", encoding="utf-8") as f:
                            content = f.read()
                        last_mtime = mtime
                        payload = {"filename": files_name, "content": content}
                        yield f"event: prompt\nid: {mtime}\ndata: {json.dumps(payload)}\n\n"
            except Exception as e:
                # forward errors to the client (optional)
                yield f"event: error\ndata: {str(e)}\n\n"
            await asyncio.sleep(1)
    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/rename-transcript")
def rename_transcript():
    try:
        latest_transcript = os.path.join(TRANSCRIPT_FOLDER, "transcript_latest.txt")

        if os.path.exists(latest_transcript):
            old_transcript = os.path.join(TRANSCRIPT_FOLDER, f"transcript_old_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
            os.rename(latest_transcript, old_transcript)
            return {"status": "success", "detail": f"Renamed latest transcript to {old_transcript}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    
    
@app.get("/initial-prompt", response_class=PlainTextResponse)
def inital_prompt():
    try:
        files = [f for f in os.listdir(INITIAL_PROMPT_FOLDER) if f.endswith(".txt")]
        if not files:
            raise HTTPException(status_code=404, detail="No initial prompt file found.")
        # Sort by modified time, newest first
        files.sort(key=lambda f: os.path.getmtime(os.path.join(INITIAL_PROMPT_FOLDER, f)), reverse=True)
        latest_file = os.path.join(INITIAL_PROMPT_FOLDER, files[0])
        with open(latest_file, "r", encoding="utf-8") as f:
            return f.read()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# New endpoint to save initial prompt
from pydantic import BaseModel
class SaveInitialPromptRequest(BaseModel):
    content: str

@app.post("/save-initial-prompt")
def save_initial_prompt(payload: SaveInitialPromptRequest):
    try:
        # Save to a new file with timestamp, or always overwrite the latest file
        files = [f for f in os.listdir(INITIAL_PROMPT_FOLDER) if f.endswith(".txt")]
        if files:
            # Overwrite the latest file
            files.sort(key=lambda f: os.path.getmtime(os.path.join(INITIAL_PROMPT_FOLDER, f)), reverse=True)
            latest_file = os.path.join(INITIAL_PROMPT_FOLDER, files[0])
        else:
            # If no file, create a new one
            latest_file = os.path.join(INITIAL_PROMPT_FOLDER, f"initial_prompt_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        with open(latest_file, "w", encoding="utf-8") as f:
            f.write(payload.content)
        return {"status": "success", "file": latest_file}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    

import asyncio
from fastapi.responses import StreamingResponse

# --- add anywhere after INITIAL_PROMPT_FOLDER is defined ---
@app.get("/initial-prompt/stream")
async def initial_prompt_stream():
    async def event_generator():
        last_mtime = 0.0
        while True:
            try:
                files = [f for f in os.listdir(INITIAL_PROMPT_FOLDER) if f.endswith(".txt")]
                if files:
                    latest = max(files, key=lambda f: os.path.getmtime(os.path.join(INITIAL_PROMPT_FOLDER, f)))
                    path = os.path.join(INITIAL_PROMPT_FOLDER, latest)
                    mtime = os.path.getmtime(path)
                    if mtime != last_mtime:
                        with open(path, "r", encoding="utf-8") as f:
                            content = f.read()
                        last_mtime = mtime
                        payload = {"filename": latest, "content": content}
                        yield f"event: prompt\nid: {mtime}\ndata: {json.dumps(payload)}\n\n"
            except Exception as e:
                # forward errors to the client (optional)
                yield f"event: error\ndata: {str(e)}\n\n"
            await asyncio.sleep(1)
    return StreamingResponse(event_generator(), media_type="text/event-stream")

@app.post("/latest-prompt")
def latest_prompt(payload: PromptPayload):
    try:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"prompts_{timestamp}.json"
        resource_dir = os.path.join(BASE_DIR, "../src/py_pubsub/resources")
        os.makedirs(resource_dir, exist_ok=True)
        full_path = os.path.join(resource_dir, filename)
        with open(full_path, "w", encoding="utf-8") as f:
            json.dump(payload.prompts, f, indent=2)
        return {"status": "success", "saved_to": full_path}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

from typing import List

@app.get("/transcript-history")
def get_transcript_history(participantId: str):
    try:
        # Get all .json files for the participant
        files = [f for f in os.listdir(HISTORY_FOLDER) 
                 if f.endswith(".json") and participantId in f]
        files.sort(
            key=lambda f: os.path.getmtime(os.path.join(HISTORY_FOLDER, f)), 
            reverse=True
        )
        history_items = []
        for fname in files:
            fpath = os.path.join(HISTORY_FOLDER, fname)
            try:
                with open(fpath, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    transcript = data.get("transcript", "")
                    prompt = data.get("prompt", {})
                    baseline = prompt.get("baseline", "")
                    history_items.append({
                        "transcript": transcript,
                        "prompt": baseline,
                        "annotations": data.get("annotations", None)
                    })
            except Exception:
                continue
        return {"history": history_items}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


import glob

class SaveHistoryRequest(BaseModel):
    participantId: str


@app.post("/save-history")
def save_history(payload: SaveHistoryRequest):
    participantId = payload.participantId
    try:
        # Get latest prompt file
        resource_dir = os.path.join(BASE_DIR, "../src/py_pubsub/resources")
        prompt_files = sorted(
            glob.glob(os.path.join(resource_dir, "prompts_*.json")),
            key=os.path.getmtime,
            reverse=True
        )
        if not prompt_files:
            raise Exception("No prompt files found.")
        with open(prompt_files[0], "r", encoding="utf-8") as f:
            prompt_data = json.load(f)

        # Get latest transcript file
        transcript_files = sorted(
            glob.glob(os.path.join(TRANSCRIPT_FOLDER, "*.txt")),
            key=os.path.getmtime,
            reverse=True
        )
        if not transcript_files:
            raise Exception("No transcript files found.")
        with open(transcript_files[0], "r", encoding="utf-8") as f:
            transcript_content = f.read()

        # Get latest analysis_result file (annotations)
        analysis_files = sorted(
            glob.glob(os.path.join(SAVE_FOLDER, "analysis_results*.txt")),
            key=os.path.getmtime,
            reverse=True
        )
        analysis_content = None
        if analysis_files:
            try:
                with open(analysis_files[0], "r", encoding="utf-8") as f:
                    import json as _json
                    analysis_content = _json.load(f)
            except Exception:
                analysis_content = None

        # Prepare JSON structure
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        history_filename = f"{participantId}_{timestamp}.json"
        history_path = os.path.join(HISTORY_FOLDER, history_filename)

        history_json = {
            "participantId": participantId,
            "timestamp": timestamp,
            "prompt": prompt_data,
            "transcript": transcript_content,
            "annotations": analysis_content
        }

        with open(history_path, "w", encoding="utf-8") as f:
            json.dump(history_json, f, indent=2, ensure_ascii=False)

        return {"status": "success", "file": history_filename}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/start-initial-prompt-conversation")
def start_initial_prompt_conversation(participantId: str):
    initial_prompt_assistant_prompt = """You are Ace, a conversational agent guide that helps users craft behavior prompts for social robots. Your job is to greet the user briefly, then immediately begin gathering information about the social robot the user is designing by asking questions one at a time.
                                    Step 1 – Information Gathering: Greet the user briefly (one short sentence) and immediately ask the first clarifying question. Ask the user targeted questions to understand: The task or scenario the robot will be involved in. The robot’s expected role in that task. Desired behavioral guidelines for the robot. The robot’s preferred communication style (tone, pacing, formality, etc.). 
                                    Step 2 Prompt Draft Creation: Based on the information gathered, generate a draft initial prompt for the robot. An effective system prompt should include: 1. specific, descriptive, and detailed instructions about the desired context, outcome, length, style, and etc. 2. what to do instead of what not to do. 3. a couple of good, positive examples when available. Always provide a response in valid JSON format. Output Format: You must return a structured JSON object with the following keys: {  "AceTalk": "TRUE", "promptDraft": "You are a conversational robot designed to ...", "AceBehavior": [ { "AceSpeechContent": "Segmented speech content here.", }, ... ], "AceFullSpeechContent": "Complete response content here."} 
                                    Rules: 1) Do not include triple backticks (json … ). 2) Always set "robotTalk": "TRUE" if a response is being delivered. 3) Include at least one entry in the "robotBehavior" list for each thinking or speaking segment. 4) The "robotFullSpeechContent" must contain the complete speech output, matching the content of all robotSpeechContent values concatenated in order. 5) Each robotBehavior segment must contain: "robotSpeechContent": a portion of Luna’s response. Always provide a draft for the socially assistive robot behavior prompt based on the information that you have received so far.
                                    YOUR ROLE REMINDER
                                    Ace’s job is to help the user design the prompt.
                                    Ace must always response in a structured JSON object with the following keys: {  "AceTalk": "TRUE", "promptDraft": "You are a conversational robot designed to ...", "AceBehavior": [ { "AceSpeechContent": "Segmented speech content here.", }, ... ], "AceFullSpeechContent": "Complete response content here."} 
                                    Never mix your own guiding role into the robot’s instructions.
                                    """
    prompts = {
        'baseline': initial_prompt_assistant_prompt,
        'model': 'GPT-4o',
        'voice': 'en-US-Chirp-HD-D'
    }

    try:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"prompts_{timestamp}.json"
        resource_dir = os.path.join(BASE_DIR, "../src/py_pubsub/resources")
        os.makedirs(resource_dir, exist_ok=True)
        full_path = os.path.join(resource_dir, filename)
        with open(full_path, "w", encoding="utf-8") as f:
            json.dump(prompts, f, indent=2)

        context = zmq.Context()
        socket = context.socket(zmq.PUB)
        socket.bind("tcp://localhost:12345")
        print("Publisher socket bound to tcp://localhost:12345")

        # message = "conversation started"
        socket.send_string(f"test")

        time.sleep(0.25)  # Allow time for subscribers to connect
        # socket.send_string(f"initial prompt conversation started;{participantId};{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        socket.send_string(f"initial prompt conversation started;{participantId};{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        print("Sent message:", "conversation started")

        return {"status": "success", "detail": "Conversation started"}

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/start-conversation")
def start_conversation(participantId: str):
    rename_transcript()
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://localhost:12345")
    print("Publisher socket bound to tcp://localhost:12345")

    # message = "conversation started"
    socket.send_string(f"test")

    time.sleep(0.25)  # Allow time for subscribers to connect
    socket.send_string(f"conversation started;{participantId};{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    print("Sent message:", "conversation started")

    return {"status": "success", "detail": "Conversation started"}


@app.post("/end-conversation")
def end_conversation():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://localhost:12345")
    print("Publisher socket bound to tcp://localhost:12345")

    # message = "conversation started"
    socket.send_string(f"test")

    time.sleep(0.25)  # Allow time for subscribers to connect
    socket.send_string("conversation ended")
    print("Sent message:", "conversation ended")


# Google Sheets
import gspread
from google.oauth2.service_account import Credentials
from fastapi import Request
from pydantic import EmailStr
from fastapi.responses import JSONResponse
from typing import List, Dict, Any

GS_CRED_PATH = os.path.join(BASE_DIR, "credentials.json")
GS_FILE_NAME = "Test Sheet"
GS_RANGE = "A:Z"

def get_sheet(tab_name):
    try:
        print("BASE_DIR:", BASE_DIR)
        print("GS_CRED_PATH:", GS_CRED_PATH)
        creds = Credentials.from_service_account_file(
            GS_CRED_PATH,
            scopes=["https://spreadsheets.google.com/feeds", "https://www.googleapis.com/auth/drive"],
        )
        gc = gspread.authorize(creds)
        print("Sheets available:", gc.openall())
        sh = gc.open(GS_FILE_NAME)
        print("Worksheets:", sh.worksheets())
        ws = sh.worksheet(tab_name)
        return ws
    except Exception as e:
        print("Error in get_sheet:", str(e))
        raise

@app.get("/participantId")
def get_participantId():
    try:
        ws = get_sheet("PreStudy")
        records = ws.get_all_records()

        row_count = len(records)+1
        return {"participantId": row_count}
    except Exception as e:
        print("Error in /participantId:", str(e))   # Add this
        raise HTTPException(status_code=500, detail=str(e))

class PreStudySubmission(BaseModel):
    startTime: str
    submissionTime: str
    timeTaken: str
    participantId: str    
    condition: str
    age: str
    gender: str
    education: str
    native: str
    proficiency: str
    major: str
    aiFamiliarity: str
    aiFrequency: str
    vaFrequency: str
    promptEngineeringExperience: str
    promptEngineeringConfidence: str
    promptEngineeringDescription: str

@app.post("/prestudy")
def post_prestudy(data: PreStudySubmission):
    try:
        ws = get_sheet("PreStudy")
        row = [
            data.participantId,
            data.startTime,
            data.submissionTime,
            data.timeTaken,
            data.condition,
            data.age,
            data.gender,
            data.education,
            data.native,
            data.proficiency,
            data.major,
            data.aiFamiliarity,
            data.aiFrequency,
            data.vaFrequency,
            data.promptEngineeringExperience,
            data.promptEngineeringConfidence,
            data.promptEngineeringDescription,
        ]
        ws.append_row(row)
        return {"status": "success"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

class MainStudySubmission(BaseModel):
    participantId: str
    initialPromptAssistant_landingTime: str
    initialPromptAssistant_EndTime: str
    initialPromptAssistant_TimeTaken: str
    mainTask_landingTime: str
    mainTask_EndTime: str
    mainTask_TimeTaken: str
    savePromptClicks: int
    startClicks: int
    endClicks: int
    numCycles: int
    satisfactionScore: List[str]

@app.post("/main-study")
def post_mainstudy(data: MainStudySubmission):
    try:
        ws = get_sheet("StudyData")
        row = [
            data.participantId,
            data.initialPromptAssistant_landingTime,
            data.initialPromptAssistant_EndTime,
            data.initialPromptAssistant_TimeTaken,
            data.mainTask_landingTime,
            data.mainTask_EndTime,
            data.mainTask_TimeTaken,
            data.savePromptClicks,
            data.startClicks,
            data.endClicks,
            data.numCycles,
            ", ".join(data.satisfactionScore),
        ]
        ws.append_row(row)
        return {"status": "success"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

from pydantic import Field
from typing import Optional

class PostStudySubmission(BaseModel):
    # Meta info
    participantId: str
    condition: str
    landingTime: str
    submissionTime: str
    timeTaken: str

    satisfied_final: str
    confident_final: str
    expectations: str
    confident_future: str
    user_satisfied: str
    not_follow_design: str

    luna_easy: str
    luna_not_understand: str
    luna_enjoy: str
    luna_again: str

    ace_freq: str
    ace_complex: str
    ace_easy: str
    ace_support: str
    ace_integrated: str
    ace_inconsistent: str
    ace_learnquick: str
    ace_cumbersome: str
    ace_confident: str
    ace_learnbefore: str

    class Config:
        allow_population_by_field_name = True
        

@app.post("/poststudy")
def post_poststudy(data: PostStudySubmission):
    try:
        ws = get_sheet("PostStudy")
        row = [
            data.participantId,
            data.condition,
            data.landingTime,
            data.submissionTime,
            data.timeTaken,
            data.satisfied_final,
            data.confident_final,
            data.expectations,
            data.confident_future,
            data.user_satisfied,
            data.not_follow_design,
            data.luna_easy,
            data.luna_not_understand,
            data.luna_enjoy,
            data.luna_again,
            data.ace_freq,
            data.ace_complex,
            data.ace_easy,
            data.ace_support,
            data.ace_integrated,
            data.ace_inconsistent,
            data.ace_learnquick,
            data.ace_cumbersome,
            data.ace_confident,
            data.ace_learnbefore,
        ]
        ws.append_row(row)
        return {"status": "success"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

    
class CompensationSubmission(BaseModel):
    submissionTime: str
    first: str
    last: str
    email: str
    confirmEmail: str

@app.post("/compensation")
def post_compensation(data: CompensationSubmission):
    try:
        ws = get_sheet("Compensation")
        
        row = [
            data.submissionTime,
            data.first,
            data.last,
            data.email,
            data.confirmEmail,
        ]
        ws.append_row(row)
        return {"status": "success"}
    except Exception as e:
        print("Error in /compensation:", str(e))
        raise HTTPException(status_code=500, detail=str(e))

class SessionLog(BaseModel):
    participantId: str = ""
    prompt: str = ""
    model: str = ""
    voice: str = ""
    userFeedback: str = ""
    aiSuggestedPromptRaw: str = ""
    aiSuggestedPromptEdited: str = ""
    buttonTimestamps: list = []
    allPrompts: List[Dict[str, Any]] = []
    allAnnotationSummaries: List[Dict[str, Any]] = []
    allAiSuggestedPrompts: List[Dict[str, Any]] = []
    satisfactionScore: List[str] = []

@app.post("/session-log-condition1")
async def save_session_log(log: SessionLog):
    log_file = os.path.join(LOG_FOLDER, f"{log.participantId}_session.txt")
    with open(log_file, "w", encoding="utf-8") as f:
        f.write(f"Participant ID: {log.participantId}\n")
        # f.write(f"Final Model: {log.model}\n")
        # f.write(f"Final Voice: {log.voice}\n")
        f.write(f"Final Prompt: {log.prompt}\n")
        f.write("Button Timestamps:\n")
        for entry in log.buttonTimestamps:
            btn = entry.get("button", "?")
            ts = entry.get("time", "?")
            f.write(f"  - {btn}: {ts}\n")
        f.write("All model configs:\n")
        for entry in log.allPrompts:
            time = entry.get("time", "?")
            # model = entry.get("model", "?")
            # voice = entry.get("voice", "?")
            prompt = entry.get("prompt", "")
            f.write(f"  - [{time}]: {prompt}\n")
        f.write("Satisfaction Scores:\n")
        for idx, score in enumerate(log.satisfactionScore):
            f.write(f"  - Cycle {idx+1}: {score}\n")
    return JSONResponse({"status": "success", "file": log_file})


@app.post("/session-log-condition2")
async def save_session_log(log: SessionLog):
    log_file = os.path.join(LOG_FOLDER, f"{log.participantId}_session.txt")
    with open(log_file, "w", encoding="utf-8") as f:
        f.write(f"Participant ID: {log.participantId}\n")
        f.write(f"Final Model: {log.model}\n")
        f.write(f"Final Voice: {log.voice}\n")
        f.write(f"Final Prompt: {log.prompt}\n")
        f.write(f"Last User Feedback: {log.userFeedback}\n")
        f.write(f"Last AI Prompt Suggestion: {log.aiSuggestedPromptRaw}\n")
        f.write(f"Last User Edited AI Prompt Suggestion: {log.aiSuggestedPromptEdited}\n")
        f.write("Button Timestamps:\n")
        for entry in log.buttonTimestamps:
            btn = entry.get("button", "?")
            ts = entry.get("time", "?")
            f.write(f"  - {btn}: {ts}\n")
        f.write("All model configs:\n")
        for entry in log.allPrompts:
            time = entry.get("time", "?")
            model = entry.get("model", "?")
            voice = entry.get("voice", "?")
            prompt = entry.get("prompt", "")
            f.write(f"  - [{time}] {model}, {voice}: {prompt}\n")
        f.write("All User Feedback Summary:\n")
        for entry in log.allUserFeedback:
            time = entry.get("time", "?")
            feedback = entry.get("feedback", "")
            f.write(f"  - [{time}] User Feedback: {feedback}\n")
        f.write("All AI Suggested Prompts:\n")
        for entry in log.allAiSuggestedPrompts:
            time = entry.get("time", "?")
            prompt = entry.get("prompt", "")
            f.write(f"  - [{time}] AI Suggested Prompt: {prompt}\n")
        f.write("All AI Suggested Prompts (Edited):\n")
        for entry in log.allAiSuggestedPromptsEdited:
            time = entry.get("time", "?")
            prompt = entry.get("prompt", "")
            f.write(f"  - [{time}] AI Suggested Prompt (Edited): {prompt}\n")
    return JSONResponse({"status": "success", "file": log_file})


@app.post("/session-log-condition3")
async def save_session_log(log: SessionLog):
    log_file = os.path.join(LOG_FOLDER, f"{log.participantId}_session.txt")
    with open(log_file, "w", encoding="utf-8") as f:
        f.write(f"Participant ID: {log.participantId}\n")
        f.write(f"Final Model: {log.model}\n")
        f.write(f"Final Voice: {log.voice}\n")
        f.write(f"Final Prompt: {log.prompt}\n")
        # f.write(f"Transcript: {log.transcript}\n")
        # f.write(f"Annotation: {log.annotation}\n")
        # f.write(f"Annotation Summary: {log.annotationSummary}\n")
        # f.write(f"AI Suggested Prompt (Raw): {log.aiSuggestedPromptRaw}\n")
        # f.write(f"AI Suggested Prompt (Edited): {log.aiSuggestedPromptEdited}\n")
        f.write("Button Timestamps:\n")
        for entry in log.buttonTimestamps:
            btn = entry.get("button", "?")
            ts = entry.get("time", "?")
            f.write(f"  - {btn}: {ts}\n")
        f.write("All model configs:\n")
        for entry in log.allPrompts:
            time = entry.get("time", "?")
            model = entry.get("model", "?")
            voice = entry.get("voice", "?")
            prompt = entry.get("prompt", "")
            f.write(f"  - [{time}] {model}, {voice}: {prompt}\n")
    return JSONResponse({"status": "success", "file": log_file})


@app.post("/session-log-experimental")
async def save_session_log(log: SessionLog):
    log_file = os.path.join(LOG_FOLDER, f"{log.participantId}_session.txt")
    with open(log_file, "w", encoding="utf-8") as f:
        f.write(f"Participant ID: {log.participantId}\n")
        f.write(f"Final Model: {log.model}\n")
        f.write(f"Final Voice: {log.voice}\n")
        f.write(f"Final Prompt: {log.prompt}\n")
        f.write(f"Last User Feedback: {log.userFeedback}\n")
        f.write(f"Last AI Prompt Suggestion: {log.aiSuggestedPromptRaw}\n")
        f.write("Button Timestamps:\n")
        for entry in log.buttonTimestamps:
            btn = entry.get("button", "?")
            ts = entry.get("time", "?")
            f.write(f"  - {btn}: {ts}\n")
        f.write("All model configs:\n")
        for entry in log.allPrompts:
            time = entry.get("time", "?")
            model = entry.get("model", "?")
            voice = entry.get("voice", "?")
            prompt = entry.get("prompt", "")
            f.write(f"  - [{time}] {model}, {voice}: {prompt}\n")
        f.write("All User Feedback Summaries:\n")
        for entry in log.allAnnotationSummaries:
            time = entry.get("time", "?")
            feedback = entry.get("feedback", "")
            f.write(f"  - [{time}] User Feedback: {feedback}\n")
        f.write("All AI Suggested Prompts:\n")
        for entry in log.allAiSuggestedPrompts:
            time = entry.get("time", "?")
            prompt = entry.get("prompt", "")
            f.write(f"  - [{time}] AI Suggested Prompt: {prompt}\n")
        f.write("Satisfaction Scores:\n")
        for idx, score in enumerate(log.satisfactionScore):
            f.write(f"  - Cycle {idx+1}: {score}\n")
    return JSONResponse({"status": "success", "file": log_file})


import openai
import os
from fastapi import FastAPI, HTTPException, Body

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

PROMPT_FILE = os.path.join(os.path.dirname(__file__), "openai_prompts.json")
try:
    with open(PROMPT_FILE, "r", encoding="utf-8") as f:
        prompt_json = json.load(f)
        SYSTEM_PROMPT = prompt_json.get("system_prompt", "")
except Exception as e:
    print("Failed to load system prompt:", e)
    SYSTEM_PROMPT = ""  # Fallback

@app.post("/generate-ai-prompt")
async def generate_ai_prompt(data: dict = Body(...)):
    editableSuggestions = data.get("editableSuggestions", "")
    # Get latest prompt and append to editableSuggestions
    resource_dir = os.path.join(BASE_DIR, "../src/py_pubsub/resources")
    try:
        prompt_files = sorted(
            glob.glob(os.path.join(resource_dir, "prompts_*.json")),
            key=os.path.getmtime,
            reverse=True
        )
        if prompt_files:
            with open(prompt_files[0], "r", encoding="utf-8") as f:
                prompt_data = json.load(f)
            # Use the baseline prompt if available, else the whole dict as string
            latest_prompt = prompt_data.get("baseline") or json.dumps(prompt_data)
        else:
            latest_prompt = ""
    except Exception as e:
        latest_prompt = ""

    if not editableSuggestions:
        raise HTTPException(status_code=400, detail="No user feedback provided.")
    if not OPENAI_API_KEY:
        raise HTTPException(status_code=500, detail="OpenAI API key not set.")
    try:
        client = openai.OpenAI(api_key=OPENAI_API_KEY)
        response = client.chat.completions.create(
            model="gpt-4o-mini-2024-07-18",
            messages=[
                {"role": "system", "content": "You are an assistant that creates an effective and optimized system prompt for a conversational robot using user feedback and the system prompt used in the previous conversation. Your task is to improve the system prompt so it reflects the user's intent and incorporates relevant feedback. Preserve critical behavioral, formatting, and operational rules from the previous prompt unless the feedback specifies otherwise. An effective system prompt should include: 1. specific, descriptive, and detailed instructions about the desired context, outcome, length, style, and etc. 2. what to do instead of what not to do. 3. a couple of good, positive examples when available. Your output should be a single, coherent system prompt to be used by the conversational robot. Do not include special characters (like # or *) in your output. Do not include headings."},
                {"role": "user", "content": editableSuggestions + ("\n" + latest_prompt if latest_prompt else "")}
            ],
            max_tokens=2500,
            temperature=0.7,
        )
        ai_prompt = response.choices[0].message.content.strip()
        return {"aiPrompt": ai_prompt}
    except Exception as e:
        print("OpenAI error:", e)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/generate-feedback-summary")
async def generate_feedback_summary():
    # Find the latest analysis_result file in the analysis_results folder
    annotations = ""
    try:
        files = glob.glob(os.path.join(SAVE_FOLDER, "*.txt"))
        if not files:
            raise HTTPException(status_code=404, detail="No analysis result files found.")
        files.sort(key=lambda f: os.path.getmtime(f), reverse=True)
        latest_file = files[0]
        # log the name of the latest file
        print("Latest analysis result file:", latest_file)
        with open(latest_file, "r", encoding="utf-8") as f:
            annotations = f.read()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to read analysis result: {e}")

    if not annotations:
        raise HTTPException(status_code=400, detail="No user feedback provided.")
    if not OPENAI_API_KEY:
        raise HTTPException(status_code=500, detail="OpenAI API key not set.")
    try:
        client = openai.OpenAI(api_key=OPENAI_API_KEY)
        response = client.chat.completions.create(
            model="gpt-4o-mini-2024-07-18",
            messages=[
                {"role": "system", "content": "You are an assistant that summarizes user feedback for improving a social robot’s behavior based on conversation transcript with user feedback annotations. The user feedback annotations will include user's marking of which parts of the conversation were liked and unliked with corresponding tags or comments. Your task is to analyze the transcript and annotations. Identify patterns in what the user considers important, unimportant, or enjoyable. Translate these insights into clear behavioral adjustment recommendations for the robot when interacting with this specific user. Produce a clear and concise structured feedback summary with the following: (1) essential behaviors to maintain (2) behaviors to reduce or avoid (3) positive engagement cues (4) recommended adjustments. Provide essential behaviors to maintain, behaviors to reduce or avoid, positive engagement cues, and recommended adjustments as bulleted lists. Include a few positive examples that users liked when available to help demonstrate preferred robot behaviors. Do not include special characters in your output outside of headings. Do not include headings other than \"# Essential Behaviors to Maintain\", \"# Behaviors to Reduce or Avoid\", \"# Positive Engagement Cues\", and \"# Recommended Adjustments\"."},
                {"role": "user", "content": annotations}
            ],
            max_tokens=2500,
            temperature=0.7,
        )
        ai_prompt = response.choices[0].message.content.strip()
        return {"aiPrompt": ai_prompt}
    except Exception as e:
        print("OpenAI error:", e)
        raise HTTPException(status_code=500, detail=str(e))
    
