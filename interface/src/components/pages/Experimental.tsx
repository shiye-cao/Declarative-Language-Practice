import React, { useState, useRef, useCallback, useEffect } from "react";
import TranscriptAnalysis from "../TranscriptAnalysis";
import ModelConfig from "../ModelConfig";
import AIFeedbackPrompt from "../AIFeedbackPrompt";
import { useNavigate } from "react-router-dom";

type TimestampEntry = { button: string; time: string };

const defaultSessionLog = {
  participantId: "",
  prompt: "",
  model: "",
  voice: "",
  buttonTimestamps: [] as TimestampEntry[],
  allPrompts: [] as { prompt: string; model: string; voice: string; time: string }[],
  allAnnotationSummaries: [] as { feedback: string; time: string }[],
  allAiSuggestedPrompts: [] as { prompt: string; time: string }[],
  satisfactionScore: [] as string[], // NEW
};

const Experimental: React.FC = () => {
  // Record landing time in localStorage
  React.useEffect(() => {
    const now = new Date().toISOString();
    localStorage.setItem("mainTask_landingTime", now);
  }, []);
  // Track if history has been saved for the current cycle
  const [historySavedForCycle, setHistorySavedForCycle] = useState(false);
  // Prompt/model/voice config state
  const [allPrompts, setAllPrompts] = useState<{prompt: string, model: string, voice: string, time: string}[]>([]);
  const [currentPrompt, setCurrentPrompt] = useState("");
  const [currentModel, setCurrentModel] = useState("gpt-4o");
  const [currentVoice, setCurrentVoice] = useState("en-US-Chirp-HD-F");
  const [progress, setProgress] = useState({
    promptSaved: false,
    conversationStarted: false,
    conversationEnded: false,
    surveySubmitted: false, // NEW
    feedbackProcessed: false,
  });
  const [buttonClicks, setButtonClicks] = useState({
    savePrompt: 0,
    start: 0,
    end: 0,
    voicePreview: 0,
    generatePrompt: 0,
    saveGeneratedPrompt: 0,
  });
  const [cycleCount, setCycleCount] = useState(0);
  const [submitting, setSubmitting] = useState(false);
  const [clearHighlightsTrigger, setClearHighlightsTrigger] = useState(0);

  // Session log state for txt (full session info)
  const [sessionLog, setSessionLog] = useState(() => {
    const pid = localStorage.getItem("participantId") || "";
    return { ...defaultSessionLog, participantId: pid };
  });
  const [generatedPrompt, setGeneratedPrompt] = useState("");
  const [annotationSummary, setAnnotationSummary] = useState("");
  const [suggestions, setSuggestions] = useState<string[]>([]);
  const [transcriptKey, setTranscriptKey] = useState(0);
  const [feedbackKey, setFeedbackKey] = useState(0);
  const [transcript, setTranscript] = useState("");
  // const [selectedBehavior, setSelectedBehavior] = useState<{ facialExpression?: string; movement?: string }>({});
  const navigate = useNavigate();

  const [showSurvey, setShowSurvey] = useState(false);
  const [surveyAnswer, setSurveyAnswer] = useState<string | number>("");

  const [showHistory, setShowHistory] = useState(false);
  const [loadingHistory, setLoadingHistory] = useState(false);

  type HistoryItem = { transcript: string; prompt: string; annotations?: Record<string, string[]> | null };
  const [history, setHistory] = useState<HistoryItem[]>([]);


  // Ref to track if we're inside a cycle
  const inCycleRef = useRef(false);

  useEffect(() => {
    // if (showSurvey) setSurveyAnswer("50");
  }, [showSurvey, sessionLog.satisfactionScore]);

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
      // conversationStarted: false,
      // conversationEnded: false,
    }));
    setSessionLog((prev) => ({ ...prev, model, voice, prompt }));
    setAllPrompts((prev) => [
      ...prev,
      { prompt, model, voice, time: new Date().toISOString() }
    ]);
    inCycleRef.current = false;
  }, []);

  // Handle "Start Conversation"
  const handleStart = async () => {
    logButtonClick("start");
    // Save history for the previous cycle if not already saved
    if (!historySavedForCycle) {
      try {
        const participantId = sessionLog.participantId || localStorage.getItem("participantId") || "";
        await fetch("http://localhost:8000/save-history", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ participantId }),
        });
        setHistorySavedForCycle(true);
      } catch (err) {
        // Optionally log error, but don't block conversation start
        console.error("Failed to save history before new cycle:", err);
      }
    }
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
      }));
      inCycleRef.current = true;
      setFeedbackKey((k) => k + 1);
      setTranscript("");
      setGeneratedPrompt("");
      setAnnotationSummary("");
      setSuggestions([]);
      setTranscriptKey((k) => k + 1);
      setHistorySavedForCycle(false); // Reset for the new cycle
    } catch (error) {
      alert("Failed to start conversation.");
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
      setProgress((prev) => ({ ...prev, conversationEnded: true, conversationStarted: false }));
      if (inCycleRef.current) {
        setCycleCount((count) => count + 1);
        inCycleRef.current = false;
      }
    } catch (error) {
      alert("Failed to end conversation.");
      console.error("Error publishing prompt:", error);
    }

    setProgress((prev) => ({ ...prev, conversationEnded: true }));
    if (inCycleRef.current) {
      setCycleCount((count) => count + 1);
      inCycleRef.current = false;
    }
    setSurveyAnswer("");
    setShowSurvey(true); // NEW: show survey popup after each conversation
    setHistorySavedForCycle(false); // Allow history to be saved for the next cycle
  };

  const handleDone = async (e: { preventDefault: () => void }) => {
    e.preventDefault();

    // Require at least one conversation cycle before allowing submission
    if (cycleCount === 0) {
      alert("Please test the conversation you designed at least once.");
      return;
    }

    // If survey has not been submitted, show it and block navigation/posting
    if (!progress.conversationEnded) {
      alert("Please end conversation first");
      return;
    }

    const confirmed = window.confirm("Are you sure you want to move on?");
    if (!confirmed) return;

    setSubmitting(true);
    const startingTime = localStorage.getItem("mainTask_landingTime");
    const submissionTime = new Date().toISOString();

    const timeTaken = startingTime
      ? ((new Date(submissionTime).getTime() - new Date(startingTime).getTime()) / 1000).toString()
      : "0";

    const payload = {
      participantId: sessionLog.participantId,
      initialPromptAssistant_landingTime: localStorage.getItem("initialPromptAssistant_landingTime"),
      initialPromptAssistant_EndTime: localStorage.getItem("initialPromptAssistant_EndTime"),
      initialPromptAssistant_TimeTaken: localStorage.getItem("initialPromptAssistant_TimeTaken"),
      mainTask_landingTime: localStorage.getItem("mainTask_landingTime"),
      mainTask_EndTime: submissionTime,
      mainTask_TimeTaken: timeTaken,
      savePromptClicks: buttonClicks.savePrompt,
      startClicks: buttonClicks.start,
      endClicks: buttonClicks.end,
      numCycles: cycleCount,
      satisfactionScore: sessionLog.satisfactionScore, // NEW
    };

    try {
      await fetch("http://localhost:8000/main-study", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
    } catch (err) {
      alert("Failed to sync with sheet, but data saved locally.");
      console.error(err);
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
    
    navigate("/poststudy");
    setSubmitting(false);
  };

  // Handle generated prompt from AIFeedbackPrompt
  const handleGeneratedPrompt = (prompt: string) => {
    setGeneratedPrompt(prompt);
    setSessionLog((prev) => ({
      ...prev,
      allAiSuggestedPrompts: [
        ...(prev.allAiSuggestedPrompts || []),
        { prompt, time: new Date().toISOString() }
      ]
    }));
    logButtonClick("generatePrompt");
  };

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

  // Handle Save Prompt from AIFeedbackPrompt (move to ModelConfig)
  const handleSavePromptToConfig = (prompt?: string) => {
    // If prompt is provided, use it; otherwise fallback to generatedPrompt
    const newPrompt = prompt !== undefined ? prompt : generatedPrompt;
    setCurrentPrompt(newPrompt);
    setProgress((prev) => ({ ...prev, promptSaved: false }));
    setSessionLog((prev) => ({
      ...prev,
      allPrompts: [
        ...(prev.allPrompts || []),
        { prompt: newPrompt, model: currentModel, voice: currentVoice, time: new Date().toISOString() }
      ]
    }));
    logButtonClick("saveGeneratedPrompt");
  };

  // Handle annotation summary change
  const handleAnnotationSummaryChange = (summary: string) => {
    setAnnotationSummary(summary);
    setSessionLog((prev) => ({
      ...prev,
      allAnnotationSummaries: [
        ...(prev.allAnnotationSummaries || []),
        { feedback: summary, time: new Date().toISOString() }
      ]
    }));
  };

  const handleSubmit = async () => {
    setShowSurvey(false);
    if (!surveyAnswer) return;

    setProgress((prev) => ({ ...prev, surveySubmitted: true }));
    setSessionLog(prev => ({
      ...prev,
      satisfactionScore: [...(prev.satisfactionScore || []), String(surveyAnswer)],
    }));

    try {
      const res = await fetch("http://localhost:8000/latest-transcript");
      if (res.ok) {
        const text = await res.text();
        setTranscript(text);
      } else {
        setTranscript("");
      }
    } catch {
      setTranscript("");
    }
  };

  // === AUTO-SAVE SESSION LOG TO TXT (backend and localStorage) ===
  
  useEffect(() => {
    if (!sessionLog.participantId) return;
    const updatedLog = { ...sessionLog, allPrompts };
    fetch("http://localhost:8000/session-log-experimental", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(updatedLog),
    }).catch((err) => {
      console.error("Session log sync failed", err);
    });
    localStorage.setItem("sessionLogExperimental", JSON.stringify(updatedLog));
  }, [sessionLog, allPrompts]);

  // Prefill the prompt box with the latest file from initial_prompts when the page loads
  useEffect(() => {
    let aborted = false;

    const loadLatestInitialPrompt = async () => {
      try {
        const res = await fetch("http://localhost:8000/initial-prompt");
        if (!res.ok) throw new Error("Failed to load latest initial prompt file.");
        const text = await res.text();
        if (!aborted) {
          setCurrentPrompt(text || "");
          // Optional: if you want it to count as “saved” immediately, uncomment:
          // setProgress((p) => ({ ...p, promptSaved: true }));
        }
      } catch (err) {
        console.warn("Could not prefill initial prompt:", err);
        // leave currentPrompt as-is
      }
    };

    loadLatestInitialPrompt();
    return () => { aborted = true; };
  }, []);

  
  // === MAIN UI ===

  const getInstruction = () => {
    if (!progress.promptSaved) return 'Provide instruction prompt. Press "Save Prompt".';
    if (!progress.conversationStarted) return 'Test your instruction prompt. Press "Start Testing".';
    if (!progress.conversationEnded) return 'To end testing, press "End Tesing".';
    if (progress.conversationEnded && !('surveySubmitted' in progress ? progress.surveySubmitted : false)) {
      return "Please fill out the survey.";
    }
    return "Review AI suggestions, make changes if needed, and save the prompt.";
  };

  return (
    
    <div className="w-full h-full">
      <div className="p-4 border rounded bg-blue-100 text-sm mb-5 max-w-5xl mx-auto">
        <strong>Instruction:</strong> {getInstruction()}
      </div>
      <div className="flex flex-row gap-3">
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
          {/* <BehavioralDisplay
            facialExpression={selectedBehavior.facialExpression}
            movement={selectedBehavior.movement}
          /> */}
        </div>
        <div className="flex-6 flex flex-col gap-1">
          <div className="flex gap-3 overflow-hidden">
            <div className="flex-1 overflow-auto">
              <TranscriptAnalysis
                transcript={transcript}
                key={transcriptKey}
                updateProgress={setProgress}
                progress={progress}
                setSuggestions={setSuggestions}
                clearHighlightsTrigger={clearHighlightsTrigger}
                // onBehaviorClick={setSelectedBehavior}
              />
            </div>
            <div className="flex-1 overflow-auto">
              <AIFeedbackPrompt
                key={feedbackKey}
                suggestions={suggestions}
                onGeneratedPrompt={handleGeneratedPrompt}
                onSavePrompt={handleSavePromptToConfig}
                annotationSummary={annotationSummary}
                onAnnotationSummaryChange={handleAnnotationSummaryChange}
                logButtonClick={logButtonClick}
              />
            </div>
          </div>

          <div className="flex justify-end mb-3">
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
      <button
        onClick={handleDone}
        className="px-10 py-3 bg-blue-600 text-white rounded py-2 mt-2 hover:bg-blue-700 disabled:opacity-50 text-sm"
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
              <span className="text-xs text-gray-500 mr-13">(Oldest to most recent)</span>
            </div>

            {loadingHistory ? (
              <div className="text-gray-500">Loading...</div>
            ) : history.length === 0 ? (
              <div className="text-gray-500">No previous transcripts found.</div>
            ) : (
              <>
                <ul className="space-y-3 max-h-80 overflow-y-auto text-xs">
                  {history.slice().reverse().map((item, i) => {
                    let annotationObj: Record<string, string[]> | null = null;
                    console.log("item:", item); // <-- why is this null?
                    if (item.annotations) {
                      if (typeof item.annotations === "string") {
                        try {
                          annotationObj = JSON.parse(item.annotations);
                        } catch {
                          annotationObj = null;
                        }
                      } else {
                        annotationObj = item.annotations;
                      }
                    }
                    return (
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
                        <hr className="border-t border-gray-300 my-2" />
                        {annotationObj && (
                          <div className="mt-2">
                            <strong>User Annotations:</strong>
                            <div>
                              <span className="font-semibold">Liked:</span>
                              <ul className="ml-4 list-disc">
                                {annotationObj["user-liked"]?.map((txt, idx) => (
                                  <li key={idx} className="text-green-700">{txt}</li>
                                ))}
                              </ul>
                            </div>
                            <div>
                              <span className="font-semibold">Disliked:</span>
                              <ul className="ml-4 list-disc">
                                {annotationObj["user-disliked"]?.map((txt, idx) => (
                                  <li key={idx} className="text-red-700">{txt}</li>
                                ))}
                              </ul>
                            </div>
                          </div>
                        )}
                      </li>
                    );
                  })}
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
                <div className="flex items-center mt-4">
                  <span className="text-xs w-6 text-left">0</span>
                  <input
                    type="range"
                    min="0"
                    max="100"
                    step="1"
                    className="w-full mx-2"
                    value={surveyAnswer === "" ? 0 : surveyAnswer}
                    onChange={e => setSurveyAnswer(Number(e.target.value))}
                  />
                  <span className="text-xs w-6 text-right">100</span>
                </div>
                <div className="text-center mt-2 text-lg font-bold">
                  {surveyAnswer !== "" && surveyAnswer}
                </div>
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

export default Experimental;