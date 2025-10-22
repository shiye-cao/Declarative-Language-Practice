import { useEffect, useState, forwardRef, useImperativeHandle } from "react";

const TranscriptViewer = forwardRef(({ progress, clearTranscriptTrigger }: {
  progress: any;
  clearTranscriptTrigger?: number;
}, ref) => {
  const [content, setContent] = useState("");
  const [loading, setLoading] = useState(false);
  const [promptText, setPromptText] = useState("");
  const [isSaving, setIsSaving] = useState(false);
  const [saveStatus, setSaveStatus] = useState<string | null>(null);

  useEffect(() => {
    // open SSE stream
    const es = new EventSource("http://localhost:8000/initial-prompt/stream");
    setLoading(true);
    const onPrompt = (evt: MessageEvent) => {
      try {
        const data = JSON.parse(evt.data);
        setContent(data.content || "");
        setPromptText(data.content || "");
        setLoading(false);
      } catch (e) {
        // ignore malformed payloads
      }
    };

    es.addEventListener("prompt", onPrompt);
    es.onerror = () => {
      // you could show a subtle warning or fall back silently
    };
    return () => {
      es.removeEventListener("prompt", onPrompt);
      es.close();
    };
  }, [progress.conversationEnded]);

  const handleSavePrompt = async () => {
    setIsSaving(true);
    setSaveStatus(null);
    try {
      const res = await fetch("http://localhost:8000/save-initial-prompt", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ content: promptText }),
      });
      if (!res.ok) throw new Error("Failed to save");
      setSaveStatus("Saved!");
    } catch (e) {
      setSaveStatus("Failed to save");
    } finally {
      setIsSaving(false);
    }
  };

  // Expose savePrompt method to parent via ref
  useImperativeHandle(ref, () => ({
    savePrompt: handleSavePrompt
  }));
  
  // Clear transcript and prompt text when trigger changes
  useEffect(() => {
    setContent("");
    setPromptText("");
  }, [clearTranscriptTrigger]);

  return (
    <div className="flex flex-col items-center justify-center">
      <div className="w-full max-w-4xl">
        <textarea
          className="w-full h-130 border border-gray-300 rounded p-2 text-sm font-mono"
          value={promptText}
          onChange={e => setPromptText(e.target.value)}
          disabled={isSaving || loading}
          placeholder="Ace will help you draft the initial prompt here."
        />
        <div className="flex items-center justify-end gap-3 mt-2">
          <button
            className="px-4 py-1 bg-blue-500 text-white rounded disabled:bg-gray-400"
            onClick={handleSavePrompt}
            disabled={isSaving || loading}
          >
            {isSaving ? "Saving..." : "Save"}
          </button>
          {saveStatus && <span className="text-xs text-gray-600">{saveStatus}</span>}
        </div>
      </div>
    </div>
  );
});

export default TranscriptViewer;