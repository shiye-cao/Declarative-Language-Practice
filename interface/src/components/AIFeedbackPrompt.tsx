import { useState, useEffect } from "react";

interface AIFeedbackPromptProps {
  suggestions: string[];
  onGeneratedPrompt: (prompt: string) => void;
  onSavePrompt: (prompt?: string) => void;
  annotationSummary: string;
  onAnnotationSummaryChange: (summary: string) => void;
  logButtonClick: (button: string) => void;
}

const AIFeedbackPrompt = ({
  suggestions,
  onGeneratedPrompt,
  onSavePrompt,
  annotationSummary,
  onAnnotationSummaryChange,
  logButtonClick,
}: AIFeedbackPromptProps) => {
  const [generatedPrompt, setGeneratedPrompt] = useState("");
  const [editableSuggestions, setEditableSuggestions] = useState(suggestions.join("\n"));
  const [feedbackProcessed, setFeedbackProcessed] = useState(suggestions.length > 0);
  const [isGenerating, setIsGenerating] = useState(false);

  useEffect(() => {
    setEditableSuggestions(suggestions.join("\n"));
    setFeedbackProcessed(suggestions.length > 0);
  }, [suggestions]);

  useEffect(() => {
    onAnnotationSummaryChange(editableSuggestions);
    // eslint-disable-next-line
  }, [editableSuggestions]);

  const handleGeneratePrompt = async () => {
    logButtonClick("generatePrompt");
    if (editableSuggestions.trim().length === 0) {
      setGeneratedPrompt("");
      return;
    }
    setIsGenerating(true);
    try {
      const res = await fetch("http://localhost:8000/generate-ai-prompt", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ editableSuggestions: editableSuggestions }),
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
    // Pass the generatedPrompt directly to onSavePrompt so Experimental can update ModelConfig immediately
    onSavePrompt(generatedPrompt);
    logButtonClick("saveGeneratedPrompt");
  };
  
  return (
    <div className="flex flex-col items-center justify-center">
      <div className="w-full max-w-6xl">
        <label className="block mb-1 font-medium">ACE Suggested Improvements</label>
        <textarea
          className="w-full h-62 p-4 border border-gray-300 rounded font-mono overflow-auto whitespace-pre-wrap text-left text-xs"
          value={editableSuggestions}
          onChange={(e) => setEditableSuggestions(e.target.value)}
          placeholder="ACE will recommend prompt adjustments based on your annotations. You may edit it before you generate suggested prompt."
        />
      </div>
      <div className="flex justify-center mb-3">
        <button
          onClick={handleGeneratePrompt}
          className={`px-4 py-2 rounded bg-gray-200 hover:bg-gray-300 ${(!feedbackProcessed || isGenerating) ? 'opacity-50 cursor-not-allowed' : ''}`}
          disabled={!feedbackProcessed || isGenerating}
        >
          {isGenerating ? "Generating..." : "Generate Suggested Prompt"}
        </button>
      </div>
      <div className="w-full max-w-6xl">
        <label className="block mb-1 font-medium">ACE Suggested Prompt</label>
        <textarea
          className="w-full h-62 p-4 border border-gray-300 rounded font-mono overflow-auto whitespace-pre-wrap text-left text-xs"
          value={generatedPrompt}
          onChange={(e) => setGeneratedPrompt(e.target.value)}
          placeholder="Ace will generate a suggested prompt based on the suggested improvements. You may edit it before saving the prompt."
        />
      </div>
      <div className="flex justify-center">
        <button
          onClick={handleSavePrompt}
          className={`px-4 py-2 rounded bg-gray-200 hover:bg-gray-300 ${!generatedPrompt ? 'opacity-50 cursor-not-allowed' : ''}`}
          disabled={!generatedPrompt}
        >
          Save Suggested Prompt
        </button>
      </div>
    </div>
  );
};

export default AIFeedbackPrompt;