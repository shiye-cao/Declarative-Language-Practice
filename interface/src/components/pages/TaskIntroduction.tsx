import React, { useState } from 'react'
import { useNavigate, useLocation } from 'react-router-dom'

const RobotInteraction: React.FC = () => {
  const navigate = useNavigate()
  const location = useLocation()
  const selectedScenarioFromState = location.state?.selectedScenario || null
  const customScenarioFromState = location.state?.customScenario || null

  const [status, setStatus] = useState("");           // shows "Child listening..."
  const [robotMessage, setRobotMessage] = useState(""); // shows response from Sam
  const [transcriptContent, setTranscriptContent] = useState("");
  const [transcriptLoading, setTranscriptLoading] = useState(false);
  const [transcriptFeedback, setTranscriptFeedback] = useState("");
  const [feedbackLoading, setFeedbackLoading] = useState(false);

  const scenarioDescriptions: Record<number, string> = {
    1: 'Scenario 1: The school bus is coming in 10 minutes. Sam\'s backpack is still unpacked. The books are sitting on the table, along with Sam\'s lunchbox. Sam also needs to put on his/her shoes and jacket before you both walk out to the bus stop.',
    2: 'Scenario 2: Sam\'s Grandma has just arrived for a visit. Grandma wants a hug from Sam. You and Sam also prepared some gifts for Grandma, a bouquet of flowers and a box of brownies, that you want Sam to give to her. '
  }

  const getSelectedScenarioText = (): string => {
    if (selectedScenarioFromState === 3 && customScenarioFromState) {
      return customScenarioFromState
    }
    return scenarioDescriptions[selectedScenarioFromState] || 'No scenario selected'
  }

  const handleStartLearning = async () => {
    // Step 1: show "Child listening..." immediately
    setStatus("Child listening...");

    try {
      // Step 2: call backend to initialize robot
      const res = await fetch("http://localhost:8000/start-child", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
      });

      if (!res.ok) throw new Error("Failed to start learning");

      const data = await res.json();
      setRobotMessage(data.robotMessage || "");
      setStatus(""); // remove "Child listening" if you want

    } catch (err) {
      console.error(err);
      setStatus("Failed to connect with Sam.");
    }
  };

  const handleEndLearning = async () => {
    // Step 1: show "Child listening..." immediately
    console.log("handleEndLearning: start")
    setStatus("Child listening...");

    try {
      // Step 2: call backend to initialize robot
      const res = await fetch("http://localhost:8000/end-practice", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
      });

      if (!res.ok) throw new Error("Failed to end practice");

      const data = await res.json();
      console.log("handleEndLearning: end-practice response", { status: res.status, ok: res.ok, data });
      //setRobotMessage(data.robotMessage || "");
      setStatus(""); // remove "Child listening" if you want

      // After ending practice, fetch latest transcript and show it in the Feedback area
      setTranscriptLoading(true);
      setTranscriptFeedback("");
      try {
        console.log("handleEndLearning: fetching latest-transcript");
        const tRes = await fetch("http://localhost:8000/latest-transcript");
        console.log("handleEndLearning: latest-transcript response", { status: tRes.status, ok: tRes.ok });
        if (!tRes.ok) throw new Error("Failed to load latest transcript");
        const text = await tRes.text();
        console.log("handleEndLearning: latest-transcript text length", text.length);
        setTranscriptContent(text);

        // Also request analysis/feedback from backend using OpenAI
        setFeedbackLoading(true);
        try {
          console.log("handleEndLearning: posting transcript to /analyze-transcript (length)", text.length);
          const aRes = await fetch("http://localhost:8000/analyze-transcript", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ transcript: text }),
          });
          console.log("handleEndLearning: analyze-transcript response", { status: aRes.status, ok: aRes.ok });
          if (!aRes.ok) {
            const errTxt = await aRes.text();
            console.error("handleEndLearning: analyze-transcript non-ok body:", errTxt);
            throw new Error(errTxt || "Failed to analyze transcript");
          }
          const aData = await aRes.json();
          console.log("handleEndLearning: analysis data", aData);
          setTranscriptFeedback(aData.feedback || "");
        } catch (aErr) {
          console.error("handleEndLearning: analysis error", aErr);
          setTranscriptFeedback(`Error analyzing transcript: ${aErr instanceof Error ? aErr.message : aErr}`);
        } finally {
          setFeedbackLoading(false);
        }

      } catch (tErr) {
        console.error("handleEndLearning: transcript fetch error", tErr);
        setTranscriptContent(`Error loading transcript. ${tErr instanceof Error ? tErr.message : tErr}`);
      } finally {
        setTranscriptLoading(false);
      }

    } catch (err) {
      console.error(err);
      setStatus("Failed to connect with Sam.");
    }
  };

  return (
    <div className="min-h-screen p-10 bg-white max-w-5xl mx-auto flex flex-col">
      {/* Setup Instructions */}
      <section className="mb-10">
        <h1 className="text-3xl font-bold mb-4">Set Up Your Robot</h1>
        <p className="text-lg text-gray-700 mb-2">
          Welcome! Before we start, make sure your robot is ready to respond to your declarative language.
        </p>
        <ul className="list-disc list-inside text-gray-700 space-y-1">
          <li>Ensure your microphone is on and working.</li>
          <li>Speak clearly in short, declarative sentences.</li>
          <li>Watch the robot respond to your language in real-time.</li>
        </ul>
      </section>

      {/* Selected Scenario Display */}
      <section className="mb-10 p-6 border border-gray-300 rounded-lg bg-gray-50">
        <h2 className="text-2xl font-semibold mb-4">Your Scenario</h2>
        <p className="text-gray-700">{getSelectedScenarioText()}</p>
      </section>

      {/* Main Interaction Area */}
      <section className="flex flex-col md:flex-row mb-10 gap-8">

        {/* Feedback area (Transcript) */}
        <div className="flex-1 border border-gray-300 rounded-lg p-6 bg-gray-50">
          <h2 className="text-xl font-semibold mb-4">Feedback</h2>
          <div className="w-full max-w-4xl">
            <label className="block mb-1 font-medium">Transcript</label>
            <div
              className="w-full h-130 p-4 border border-gray-300 rounded font-mono overflow-auto whitespace-pre-wrap text-left text-xs bg-white"
            >
              {transcriptLoading
                ? <span className="text-gray-400 italic">Loading transcript…</span>
                : (transcriptContent.trim().length === 0
                  ? <span className="text-gray-400">Your conversation transcript will appear here after you end testing.</span>
                  : transcriptContent)}
            </div>
          </div>
          <div className="mt-4 w-full max-w-4xl">
            <label className="block mb-1 font-medium">AI Feedback</label>
            <div className="w-full p-4 border border-gray-300 rounded font-sans overflow-auto whitespace-pre-wrap text-left text-sm bg-white">
              {feedbackLoading
                ? <span className="text-gray-400 italic">Analyzing transcript…</span>
                : (transcriptFeedback.trim().length === 0
                  ? <span className="text-gray-400">AI feedback will appear here after analysis.</span>
                  : transcriptFeedback)}
            </div>
          </div>
        </div>
      </section>

      {/* Bottom: Learning Buttons */}
      <div className="flex justify-center gap-4 mt-auto">
        <button
          onClick={handleStartLearning}
          className="px-6 py-3 bg-green-600 text-white rounded hover:bg-green-700 transition"
        >
          Start Learning
        </button>
        {status && <p className="text-green-600 font-semibold">{status}</p>}
        {robotMessage && (
          <div className="mt-3 p-3 border rounded bg-gray-100">
            <p>{robotMessage}</p>
          </div>
        )}
        <button
          // onClick={() => navigate('/feedback')}
          onClick={handleEndLearning}
          className="px-6 py-3 bg-blue-600 text-white rounded hover:bg-blue-700 transition"
        >
          Done Learning
        </button>
      </div>
    </div>
  )
}

export default RobotInteraction
