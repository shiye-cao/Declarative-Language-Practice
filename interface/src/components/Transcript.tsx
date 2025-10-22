import React, { useEffect, useState } from "react";

const TranscriptViewer = ({
  progress,
  clearTranscriptTrigger
}: {
  progress: any;
  clearTranscriptTrigger?: number;
}) => {
  const [content, setContent] = useState("");
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    if (!progress.surveySubmitted) return;

    setLoading(true);

    fetch("http://localhost:8000/latest-transcript")
      .then(res => {
        if (!res.ok) {
          throw new Error("Failed to load latest transcript file.");
        }
        return res.text();
      })
      .then(text => {
        setContent(text);
      })
      .catch(error => {
        setContent(
          `Error loading transcript file. Please check the backend.\nError: ${error.message}`
        );
      })
      .finally(() => setLoading(false));
  }, [progress.surveySubmitted]);

  // Clear transcript when trigger changes
  useEffect(() => {
    setContent("");
  }, [clearTranscriptTrigger]);

  return (
    <div className="flex flex-col items-center justify-center">
      <div className="w-full max-w-4xl">
        <label className="block mb-1 font-medium">Transcript</label>
        <div
          className="w-full h-130 p-4 border border-gray-300 rounded font-mono overflow-auto whitespace-pre-wrap text-left text-xs bg-white"
        >
          {loading
            ? <span className="text-gray-400 italic">Loading transcript…</span>
            : (content.trim().length === 0
              ? <span className="text-gray-400">Your conversation transcript will appear here after you end testing.</span>
              : content)}
        </div>
      </div>
    </div>
  );
};

export default TranscriptViewer;