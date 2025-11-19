import React, { useState } from 'react'
import { useNavigate } from 'react-router-dom'
//import FeedbackPrompt from '../FeedbackPrompt'

  //const feedbackProcessed = userFeedback.trim().length > 0;

const RobotInteraction: React.FC = () => {
  const navigate = useNavigate()
  const [selectedScenario, setSelectedScenario] = useState<number | null>(null)
  const [showGenerator, setShowGenerator] = useState<boolean>(false)
  const [generatorInput, setGeneratorInput] = useState<string>('')
  const [generatedPrompt, setGeneratedPrompt] = useState<string | null>(null)
  //const [status, setStatus] = useState("");         // <-- NEW
  //const [childMessage, setChildMessage] = useState(""); // <-- NEW
  //const [childStatus, setChildStatus] = useState("")
  //const [generatedPrompt, setGeneratedPrompt] = useState("");
  //const [isGenerating, setIsGenerating] = useState(false);
  const [status, setStatus] = useState("");           // shows "Child listening..."
  const [robotMessage, setRobotMessage] = useState(""); // shows response from Sam
  

  const scenarioDescriptions: Record<number, string> = {
    1: 'Scenario 1: The school bus is coming in 10 minutes. Sam’s backpack is still unpacked. The books are sitting on the table, along with Sam’s lunchbox. Sam also needs to put on his/her shoes and jacket before you both walk out to the bus stop.',
    2: 'Scenario 2: Sam’s Grandma has just arrived for a visit. Grandma wants a hug from Sam. You and Sam also prepared some gifts for Grandma, a bouquet of flowers and a box of brownies, that you want Sam to give to her. '
  }

  const toggleScenario = (n: number) => {
    setSelectedScenario(prev => (prev === n ? null : n))
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

    const handleGenerateScenarioPrompt = async () => {

    try {
      const res = await fetch("http://localhost:8000/generate-scenario-prompt", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({userPrompt: generatorInput}),
      });
      if (!res.ok) throw new Error("Failed to generate prompt");
      const data = await res.json();
      setGeneratedPrompt(data.aiPrompt || "");
    } catch (err) {
      setGeneratedPrompt("");
      alert("Failed to generate AI prompt.");
      console.error(err);
    }
  };
  //{childStatus && <p className="text-green-600 mt-2">{childStatus}</p>}

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

      {/* Row: Scenarios & Popup */}
      <section className="mb-6">
        <div className="flex flex-col sm:flex-row gap-4 items-start">
          {/* Scenario 1 Card */}
          <div
            className={`w-full sm:w-1/3 border rounded-lg p-4 cursor-pointer transition-shadow ${
              selectedScenario === 1 ? 'border-blue-500 bg-blue-50 shadow' : 'border-gray-200 bg-white'
            }`}
            onClick={() => toggleScenario(1)}
            role="button"
            tabIndex={0}
            onKeyDown={(e) => { if (e.key === 'Enter' || e.key === ' ') toggleScenario(1) }}
            aria-pressed={selectedScenario === 1}
          >
            <div className="flex items-center justify-between">
              <h3 className="text-lg font-medium">Scenario 1</h3>
              <button
                className={`px-3 py-1 rounded text-sm ${selectedScenario === 1 ? 'bg-blue-600 text-white' : 'bg-gray-100 text-gray-700'}`}
                onClick={(e) => { e.stopPropagation(); toggleScenario(1) }}
                aria-label="Select Scenario 1"
              >
                Select
              </button>
            </div>
            {selectedScenario === 1 && (
              <p className="mt-3 text-gray-700 text-sm">{scenarioDescriptions[1]}</p>
            )}
          </div>

          {/* Scenario 2 Card */}
          <div
            className={`w-full sm:w-1/3 border rounded-lg p-4 cursor-pointer transition-shadow ${
              selectedScenario === 2 ? 'border-blue-500 bg-blue-50 shadow' : 'border-gray-200 bg-white'
            }`}
            onClick={() => toggleScenario(2)}
            role="button"
            tabIndex={0}
            onKeyDown={(e) => { if (e.key === 'Enter' || e.key === ' ') toggleScenario(2) }}
            aria-pressed={selectedScenario === 2}
          >
            <div className="flex items-center justify-between">
              <h3 className="text-lg font-medium">Scenario 2</h3>
              <button
                className={`px-3 py-1 rounded text-sm ${selectedScenario === 2 ? 'bg-blue-600 text-white' : 'bg-gray-100 text-gray-700'}`}
                onClick={(e) => { e.stopPropagation(); toggleScenario(2) }}
                aria-label="Select Scenario 2"
              >
                Select
              </button>
            </div>
            {selectedScenario === 2 && (
              <p className="mt-3 text-gray-700 text-sm">{scenarioDescriptions[2]}</p>
            )}
          </div>

          {/* Generate new Prompt */}
          <div
            className={`w-full sm:w-1/3 border rounded-lg p-4 transition-shadow ${
              showGenerator ? 'border-blue-500 bg-blue-50 shadow' : 'border-gray-200 bg-white cursor-pointer'
            }`}
            onClick={() => { if (!showGenerator) { setShowGenerator(true); setSelectedScenario(3) } }}
            role="button"
            tabIndex={0}
            onKeyDown={(e) => { if (e.key === 'Enter' || e.key === ' ') { setShowGenerator(prev => !prev); setSelectedScenario(prev => prev === 3 ? null : 3) } }}
            aria-pressed={showGenerator}
          >
            <div className="flex items-center justify-between">
              <h3 className="text-lg font-medium">Generate new prompt</h3>
              <button
                className={`px-3 py-1 rounded text-sm ${showGenerator ? 'bg-blue-600 text-white' : 'bg-gray-100 text-gray-700'}`}
                onClick={(e) => { e.stopPropagation(); setShowGenerator(prev => !prev); setSelectedScenario(prev => prev === 3 ? null : 3) }}
                aria-label="Generate new scenario"
              >
                {showGenerator ? 'Close' : 'Open'}
              </button>
            </div>

            {showGenerator && (
              <div className="mt-3">
                <label className="block text-sm text-gray-700 mb-1">Enter a brief description (optional)</label>
                <textarea
                  value={generatorInput}
                  onChange={(e) => setGeneratorInput(e.target.value)}
                  className="w-full border border-gray-300 rounded p-2 text-sm"
                  rows={4}
                  placeholder="Type your scenario or leave blank to auto-generate..."
                  onClick={(e) => e.stopPropagation()}
                  onKeyDown={(e) => e.stopPropagation()}
                />

                <div className="mt-3 flex items-center gap-3">
                  <button
                    onClick={(e) => {
                      e.stopPropagation()
                      handleGenerateScenarioPrompt();
                      // Hard-coded generation result for now
                      //setGeneratedPrompt('It’s early Saturday morning, and you and Sam are in the kitchen getting ready to bake cookies together. The sun is shining through the window, and you can hear birds chirping outside. The counters are all clean, and all the ingredients—flour, sugar, eggs, butter, chocolate chips—are laid out in front of you. Sam has been looking forward to this because they love chocolate chip cookies, and you want to make sure everything goes smoothly so Sam stays calm and happy during the activity. You notice Sam is quietly watching you measure the flour.')
                    }}
                    className="px-4 py-2 bg-indigo-600 text-white rounded hover:bg-indigo-700 transition text-sm"
                    aria-label="Generate prompt"
                  >
                    Generate
                  </button>

                  <button
                    onClick={(e) => { e.stopPropagation(); setGeneratorInput(''); setGeneratedPrompt(null) }}
                    className="px-3 py-2 bg-gray-100 text-gray-800 rounded hover:bg-gray-200 text-sm"
                    aria-label="Clear"
                  >
                    Clear
                  </button>
                </div>

                {generatedPrompt && (
                  <div className="mt-3 p-3 bg-white border border-gray-200 rounded">
                    <h4 className="font-medium mb-1">Generated Prompt</h4>
                    <p className="text-sm text-gray-700">{generatedPrompt}</p>
                  </div>
                )}
              </div>
            )}
          </div>
        </div>
      </section>

      {/* Main Interaction Area */}
      <section className="flex flex-col md:flex-row mb-10 gap-8">

        {/* Right: Feedback area */}
        <div className="flex-1 border border-gray-300 rounded-lg p-6 bg-gray-50">
          <h2 className="text-xl font-semibold mb-4">Feedback</h2>
          <p className="text-gray-700">
            Your robot will give feedback here based on the declarative sentences you speak. 
            For now, this is a placeholder area.
          </p>
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
          onClick={() => navigate('/feedback')}
          className="px-6 py-3 bg-blue-600 text-white rounded hover:bg-blue-700 transition"
        >
          Done Learning
        </button>
      </div>
    </div>
  )
}

export default RobotInteraction
