import React from "react";

interface TranscriptDisplayProps {
  transcript: any[];
  onBehaviorClick: (behavior: { facialExpression?: string; movement?: string }) => void;
}

const TranscriptDisplay: React.FC<TranscriptDisplayProps> = ({ transcript, onBehaviorClick }) => {
  return (
    <div className="font-mono text-xs whitespace-pre-wrap">
      {transcript.map((entry, idx) => {
        if (entry.User) {
          return (
            <div key={idx} className="mb-1">
              <span className="font-bold text-blue-700">User:</span> {entry.User.speech}
            </div>
          );
        }
        if (entry.Robot) {
          return (
            <div key={idx} className="mb-1">
              <span className="font-bold text-green-700">Robot</span>
              {" "}
              [
              <span
                className="text-purple-700 underline cursor-pointer"
                onClick={() => onBehaviorClick({ facialExpression: entry.Robot.facial_expression })}
              >
                {entry.Robot.facial_expression}
              </span>
              ]
              [
              <span
                className="text-orange-700 underline cursor-pointer"
                onClick={() => onBehaviorClick({ movement: entry.Robot.movement })}
              >
                {entry.Robot.movement}
              </span>
              ]
              : {entry.Robot.speech}
            </div>
          );
        }
        return null;
      })}
    </div>
  );
};

export default TranscriptDisplay;