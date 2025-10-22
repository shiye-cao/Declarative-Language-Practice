import { useState, useEffect } from "react";

interface FeedbackPromptProps {
  onGeneratedPrompt: (prompt: string) => void;
  onSavePrompt: () => void;
  userFeedback: string;
  onUserFeedbackChange: (feedback: string) => void;
  logButtonClick: (button: string) => void;
  progress: {
    promptSaved: boolean;
    conversationStarted: boolean;
    conversationEnded: boolean;
    feedbackProcessed: boolean;
  };
}

const FeedbackPrompt = ({
  onGeneratedPrompt,
  onSavePrompt,
  userFeedback,
  onUserFeedbackChange,
  logButtonClick,
  progress,
}: FeedbackPromptProps) => {
  const [generatedPrompt, setGeneratedPrompt] = useState("");
  const [isGenerating, setIsGenerating] = useState(false);
  const feedbackProcessed = userFeedback.trim().length > 0;

  const handleGeneratePrompt = async () => {
    logButtonClick("generatePrompt");
    if (!feedbackProcessed) {
      setGeneratedPrompt("");
      return;
    }
    setIsGenerating(true);
    try {
      const res = await fetch("http://localhost:8000/generate-ai-prompt", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ userFeedback }),
      });
      if (!res.ok) throw new Error("Failed to generate prompt");
      const data = await res.json();
      setGeneratedPrompt(data.aiPrompt || "");
    } catch (err) {
      setGeneratedPrompt("");
      alert("Failed to generate AI prompt.");
      console.error(err);
    } finally {
      setIsGenerating(false);
    }
  };

  const handleSavePrompt = async () => {
    onGeneratedPrompt(generatedPrompt);
    onSavePrompt();
  };

  return (
    <div className="flex flex-col items-center justify-center">
      <div className="w-full max-w-6xl">
        <label className="block mb-1 font-medium">User Feedback</label>
        <textarea
          className="w-full h-62 p-4 border border-gray-300 rounded font-mono overflow-auto whitespace-pre-wrap text-left text-xs"
          value={userFeedback}
          onChange={(e) => onUserFeedbackChange(e.target.value)}
          placeholder="Write your notes or feedback here."
        />
      </div>
      <div className="flex justify-center mb-3">
        <button
          onClick={handleGeneratePrompt}
          className={`px-4 py-2 rounded bg-gray-200 hover:bg-gray-300 ${(!feedbackProcessed || isGenerating) ? 'opacity-50 cursor-not-allowed' : ''}`}
          disabled={!feedbackProcessed || isGenerating}
        >
          {isGenerating ? "Generating..." : "Generate Prompt"}
        </button>
      </div>
      <div className="w-full max-w-6xl">
        <label className="block mb-1 font-medium">AI Suggested Prompt</label>
        <textarea
          className="w-full h-62 p-4 border border-gray-300 rounded font-mono overflow-auto whitespace-pre-wrap text-left text-xs"
          value={generatedPrompt}
          onChange={(e) => setGeneratedPrompt(e.target.value)}
          placeholder="Generated prompt will appear here. You can edit it before saving the prompt."
        />
      </div>
      <div className="flex justify-center">
        <button
          onClick={handleSavePrompt}
          className={`px-4 py-2 rounded bg-gray-200 hover:bg-gray-300 ${!generatedPrompt ? 'opacity-50 cursor-not-allowed' : ''}`}
          disabled={!generatedPrompt}
        >
          Save Prompt
        </button>
      </div>
    </div>
  );
};

export default FeedbackPrompt;