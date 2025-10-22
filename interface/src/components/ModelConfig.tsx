import React, { useEffect } from "react";

interface ModelConfigProps {
  onPromptSaved: (model: string, voice: string, prompt: string) => void;
  logButtonClick: (button: string) => void;
  currentPrompt: string;
  setCurrentPrompt: React.Dispatch<React.SetStateAction<string>>;
  currentModel: string;
  setCurrentModel: React.Dispatch<React.SetStateAction<string>>;
  currentVoice: string;
  setCurrentVoice: React.Dispatch<React.SetStateAction<string>>;
}

const voices = [
  { label: "Voice 1", value: "en-US-Standard-A" },
  { label: "Voice 2", value: "en-US-Wavenet-F" },
  { label: "Voice 3", value: "en-GB-Wavenet-D" },
  { label: "Voice 4", value: "en-GB-News-I" },
];

const models = [
  { label: "GPT-4o", value: "gpt-4o" },
  { label: "GPT-4o-mini", value: "gpt-4o-mini" },
];

const ModelConfig: React.FC<ModelConfigProps> = ({
  onPromptSaved,
  logButtonClick,
  currentPrompt,
  setCurrentPrompt,
  currentModel,
  setCurrentModel,
  currentVoice,
  setCurrentVoice,
}) => {
  // Voice preview

  useEffect(() => {
    if (!currentVoice) {
      setCurrentVoice(voices[0].value);
    }
    if (!currentModel) {
      setCurrentModel(models[0].value);
    }
  }, []);

  // Save prompt
  const handleSavePrompt = async () => {
    logButtonClick("savePrompt");
    if (!currentPrompt.trim()) {
      alert("Prompt cannot be empty!");
      return;
    }
    const payload = {
      prompts: {
        baseline: currentPrompt,
        model: currentModel,
        voice: currentVoice,
      },
    };
    try {
      await fetch("http://localhost:8000/latest-prompt", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      onPromptSaved(currentModel, currentVoice, currentPrompt);
      alert("Prompt saved!");
    } catch (err) {
      alert("Failed to publish prompt.");
      console.error(err);
    }
  };

  return (
    <div className="max-w-3xl mx-auto">
      <label className="block mb-1 font-medium">Prompt</label>
      <textarea
        className="w-full h-125 p-4 border border-gray-300 rounded font-mono overflow-auto whitespace-pre-wrap text-left text-xs"
        value={currentPrompt}
        onChange={(e) => setCurrentPrompt(e.target.value)}
        placeholder="Provide instructions for the robot here."
      />
      <div className="flex justify-center">
        <button
          onClick={handleSavePrompt}
          className="px-4 py-2 rounded bg-gray-200 hover:bg-gray-300"
          type="button"
        >
          Save Prompt
        </button>
      </div>
    </div>
  );
};

export default ModelConfig;