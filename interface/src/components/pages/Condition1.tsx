import React, { useState, useRef, useCallback, useEffect } from "react";
import ModelConfig from "../ModelConfig"; // Adjust the import path as necessary
import Transcript from "../Transcript"; // Adjust the import path as necessary
import { useNavigate } from "react-router-dom";

type TimestampEntry = { button: string; time: string };

const defaultSessionLog = {
  participantId: "",
  prompt: "",
  model: "",
  voice: "",
  transcript: "",
  annotation: "",
  annotationSummary: "",
  aiSuggestedPromptRaw: "",
  aiSuggestedPromptEdited: "",
  buttonTimestamps: [] as TimestampEntry[],
  allPrompts: [] as { prompt: string; model: string; voice: string; time: string }[],
  satisfactionScore: [] as string[],
};

const Condition1: React.FC = () => {
  // Prompt/model/voice config state
  const [allPrompts, setAllPrompts] = useState<{prompt: string, model: string, voice: string, time: string}[]>([]);
  const [currentPrompt, setCurrentPrompt] = useState("");
  const [currentModel, setCurrentModel] = useState("gpt-4o");
  const [currentVoice, setCurrentVoice] = useState("en-US-Standard-A");
  // Progress/cycle/button state
  const [progress, setProgress] = useState({
    promptSaved: false,
    conversationStarted: false,
    conversationEnded: false,
    surveySubmitted: false,
  });
  const [buttonClicks, setButtonClicks] = useState({
    savePrompt: 0,
    start: 0,
    end: 0,
    voicePreview: 0,
  });
  const [cycleCount, setCycleCount] = useState(0);
  const [submitting, setSubmitting] = useState(false)
  const [clearTranscriptTrigger, setClearTranscriptTrigger] = useState(0);
  const [showSurvey, setShowSurvey] = useState(false);
  const [surveyAnswer, setSurveyAnswer] = useState<string | number>("");

  // Session log state for txt (full session info)
  const [sessionLog, setSessionLog] = useState(() => {
    const pid = localStorage.getItem("participantId") || "";
    return { ...defaultSessionLog, participantId: pid };
  });
  const navigate = useNavigate();

  // Ref to track if we're inside a cycle
  const inCycleRef = useRef(false);

  // === LOGGING FUNCTIONS ===

  // Log button click and timestamp
  const logButtonClick = useCallback((button: string) => {
    setButtonClicks((prev) => ({ ...prev, [button]: (prev[button as keyof typeof prev] || 0) + 1 }));
    setSessionLog((prev) => ({
      ...prev,
      buttonTimestamps: [
        ...(prev.buttonTimestamps || []),
        { button, time: new Date().toISOString() },
      ],
    }));
  }, []);

  // Handle "Save Prompt" success
  const handlePromptSaved = useCallback((model: string, voice: string, prompt: string) => {
    setProgress((prev) => ({
      ...prev,
      promptSaved: true,
      conversationStarted: false,
      conversationEnded: false,
      surveySubmitted: false,
    }));
    setSessionLog((prev) => ({ ...prev, model, voice, prompt }));
    setAllPrompts((prev) => [
      ...prev,
      { prompt, model, voice, time: new Date().toISOString() }
    ]);
    // After saving prompt, not in cycle yet
    inCycleRef.current = false;
  }, []);

  // Handle "Start Conversation"
  const handleStart = async () => {
    logButtonClick("start");
    try {
      const participantId = sessionLog.participantId || localStorage.getItem("participantId") || "";
      
      await fetch(`http://localhost:8000/start-conversation?participantId=${encodeURIComponent(participantId)}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify("start conversation"),
      });
      setProgress((prev) => ({
        ...prev,
        conversationStarted: true,
        conversationEnded: false,
        surveySubmitted: false,
      }));
      inCycleRef.current = true;
      setClearTranscriptTrigger((prev) => prev + 1); // <-- Add this line
    } catch (error) {
      alert("Failed to publish prompt.");
      console.error("Error publishing prompt:", error);
    }
  };

  // Handle "End Conversation"
  const handleEnd = async () => {
    logButtonClick("end");
    try {
      await fetch("http://localhost:8000/end-conversation", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify("end conversation"),
      });
      setProgress((prev) => ({ ...prev, conversationEnded: true}));
      if (inCycleRef.current) {
        setCycleCount((count) => count + 1);
        inCycleRef.current = false;
      }
      setSurveyAnswer("");
      setShowSurvey(true); // <-- Show the survey popup
    } catch (error) {
      alert("Failed to end conversation.");
      console.error("Error publishing prompt:", error);
    }
  };

  // Handle "Done"
  const handleDone = async (e: { preventDefault: () => void }) => {
    // Send main study summary data to backend (for sheet)
    e.preventDefault();
    setSubmitting(true);

    const submissionTime = new Date().toISOString();

    const payload = {
      participantId: sessionLog.participantId,
      submissionTime: submissionTime,
      savePromptClicks: buttonClicks.savePrompt,
      startClicks: buttonClicks.start,
      endClicks: buttonClicks.end,
      numCycles: cycleCount,
      satisfactionScore: sessionLog.satisfactionScore,
    };
    
    try {
      await fetch("http://localhost:8000/main-study", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
    } catch (err) {
      // Still allow navigation
      alert("Failed to sync with sheet, but data saved locally.");
      console.error(err);
    }
    navigate("/poststudy");

    setSubmitting(false);
  };

  const handleSubmit = async () => {
    setShowSurvey(false);

    if (!surveyAnswer) return;

    setProgress((prev) => ({ ...prev, surveySubmitted: true}));

    // Add to session log
    setSessionLog(prev => ({
      ...prev,
      satisfactionScore: [...(prev.satisfactionScore || []), String(surveyAnswer)],
    }));

    await fetch("http://localhost:8000/save-history", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ participantId: sessionLog.participantId }),
    });
  };

  const [showHistory, setShowHistory] = useState(false);
  const [loadingHistory, setLoadingHistory] = useState(false);

  type HistoryItem = { transcript: string; prompt: string };
  const [history, setHistory] = useState<HistoryItem[]>([]);

  const handleShowHistory = async () => {
    setShowHistory(true);
    setLoadingHistory(true);
    try {
      const participantId = sessionLog.participantId || localStorage.getItem("participantId") || "";

      const res = await fetch(`http://localhost:8000/transcript-history?participantId=${encodeURIComponent(participantId)}`);
      if (!res.ok) throw new Error("Failed to fetch history");
      const data = await res.json();
      setHistory(data.history || []);
    } catch (err) {
      setHistory([]);
    }
    setLoadingHistory(false);
  };

  // === AUTO-SAVE SESSION LOG TO TXT (backend and localStorage) ===

  useEffect(() => {
    if (!sessionLog.participantId) return;
    // Add allPrompts to sessionLog before saving
    const updatedLog = { ...sessionLog, allPrompts };
    fetch("http://localhost:8000/session-log-condition1", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(updatedLog),
    }).catch((err) => {
      console.error("Session log sync failed", err);
    });
    localStorage.setItem("sessionLog", JSON.stringify(updatedLog));
  }, [sessionLog, allPrompts]);

  useEffect(() => {
    if (showSurvey) {
      setSurveyAnswer("50");
    }
  }, [showSurvey, sessionLog.satisfactionScore]);

  // === MAIN UI ===

  const getInstruction = () => {
    if (!progress.promptSaved) {
      return 'Draft the robot behavior prompt and click "Save Prompt" when finished.';
    }
    if (!progress.conversationStarted) return 'To test the robot, click "Start Conversation".';
    if (!progress.conversationEnded) return 'To end testing, click "End Conversation".';
    if (progress.conversationEnded && !progress.surveySubmitted) {
      return "Please fill out the survey.";
    }
    return 'Review the transcript and update the prompt if needed. Click "Save Prompt" when finished with update or press "Done" to complete the design process.';
  };

  return (
    <div className="w-full h-full">
      <div className="p-4 border rounded bg-blue-100 text-sm mb-5 max-w-5xl mx-auto">
        <strong>Instruction:</strong> {getInstruction()}
      </div>
      <div className="flex flex-row gap-6">
        <div className="flex-3">
          <ModelConfig
            onPromptSaved={handlePromptSaved}
            logButtonClick={logButtonClick}
            currentPrompt={currentPrompt}
            setCurrentPrompt={setCurrentPrompt}
            currentModel={currentModel}
            setCurrentModel={setCurrentModel}
            currentVoice={currentVoice}
            setCurrentVoice={setCurrentVoice}
          />
        </div>
        <div className="flex-3 flex-col gap-1">
          <Transcript progress={progress} clearTranscriptTrigger={clearTranscriptTrigger} />
          
          <div className="flex justify-end mb-4">
            <button
              className="px-3 py-2 bg-gray-600 text-white rounded mt-1 hover:bg-gray-700"
              onClick={handleShowHistory}
              type="button"
            >
              View Prompt & Transcript History
            </button>
          </div>
        </div>
      </div>
      
      <div className="p-5 flex gap-3 justify-center">
        <button
          className={`px-5 py-3 text-sm bg-green-500 hover:bg-green-600 text-white rounded ${(!progress.promptSaved || progress.conversationStarted) ? 'opacity-50 cursor-not-allowed' : ''}`}
          onClick={handleStart}
          disabled={!progress.promptSaved || progress.conversationStarted}
          type="button"
        >
          Start Testing
        </button>
        <button
          className={`px-5 py-3 text-sm bg-red-500 hover:bg-red-600 text-white rounded ${(!progress.promptSaved || !progress.conversationStarted || progress.conversationEnded) ? 'opacity-50 cursor-not-allowed' : ''}`}
          onClick={handleEnd}
          disabled={!progress.promptSaved || !progress.conversationStarted || progress.conversationEnded}
          type="button"
        >
          End Testing
        </button>
      </div>
      <button
        onClick={handleDone}
        className="px-10 py-3 bg-blue-600 text-white rounded py-2 hover:bg-blue-700 disabled:opacity-50 text-sm"
        type="submit"
        disabled={submitting}
      >
        {submitting ? 'Submitting...' : 'Done'}
      </button>

      {showHistory && (
        <div className="fixed inset-0 z-40 flex items-center justify-center">
          <div className="bg-white max-w-2xl w-full rounded shadow-lg p-6 relative text-left">
            <button
              className="absolute top-2 right-2 text-gray-600 hover:text-black"
              onClick={() => setShowHistory(false)}
            >
              ✕
            </button>
            <div className="flex items-center justify-between mb-2">
              <h2 className="text-xl font-bold">Transcript History</h2>
              <span className="text-xs text-gray-500 mr-12">(Oldest to most recent)</span>
            </div>

            {loadingHistory ? (
              <div className="text-gray-500">Loading...</div>
            ) : history.length === 0 ? (
              <div className="text-gray-500">No previous transcripts found.</div>
            ) : (
              <>
                <ul className="space-y-3 max-h-80 overflow-y-auto text-xs">
                  {history.slice().reverse().map((item, i) => (
                    <li key={i} className="border rounded p-2 bg-gray-50 whitespace-pre-wrap">
                      <div>
                        <strong>Prompt {i + 1}:</strong>
                        <br />
                        <span className="italic">{item.prompt || <em>No prompt saved</em>}</span>
                      </div>
                      <hr className="border-t border-gray-300 my-2" />
                      <div>
                        <strong>Transcript {i + 1}:</strong>
                        <br />
                        {item.transcript || <em>No transcript found</em>}
                      </div>
                    </li>
                  ))}
                </ul>
              </>
            )}
          </div>
        </div>
      )}


      {showSurvey && (
        <div className="fixed inset-0 flex items-center justify-center z-50">
          <div className="bg-white p-6 rounded shadow-lg max-w-sm w-full">
            <h2 className="text-lg font-semibold mb-4">Quick Survey</h2>
            <label className="block mb-2">
              <span>On a scale from 0 to 100, how satisfied are you with this conversation?</span>
              <input
                type="range"
                min="0"
                max="100"
                step="1"
                className="w-full mt-4"
                value={surveyAnswer === "" ? 50 : surveyAnswer} // Default to 5 if empty
                onChange={e => setSurveyAnswer(Number(e.target.value))}
              />
              <div className="text-center mt-2 text-lg font-bold">{surveyAnswer === "" ? 50 : surveyAnswer}</div>
            </label>
            <div className="flex justify-end mt-4">
              <button
                className="px-4 py-2 bg-blue-600 text-white rounded disabled:opacity-50"
                disabled={surveyAnswer === ""}
                onClick={handleSubmit}
              >
                Submit
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default Condition1;