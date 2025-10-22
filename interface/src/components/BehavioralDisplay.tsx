import React from "react";

interface BehavioralDisplayProps {
  facialExpression?: string;
  movement?: string;
}

const facialExpressionMap: Record<string, string> = {
  neutral: "/behaviors/facial/neutral.png",
  satisfied: "/behaviors/facial/satisfied.png",
  happy: "/behaviors/facial/happy.png",
  surprised: "/behaviors/facial/surprised.png",
  interested: "/behaviors/facial/interested.png",
  excited: "/behaviors/facial/excited.png",
};

const movementMap: Record<string, string> = {
  nod: "/behaviors/movement/nod.gif",
  doubleNod: "/behaviors/movement/doubleNod.gif",
  lookAtUser: "/behaviors/movement/lookAtUser.gif",
};

const BehavioralDisplay: React.FC<BehavioralDisplayProps> = ({ facialExpression, movement }) => {
  if (!facialExpression && !movement) return null;
  return (
    <div className="flex flex-col items-center my-4">
      {facialExpression && (
        <div className="mb-2">
          <div className="text-xs text-gray-500 text-center mb-1">Facial Expression: {facialExpression}</div>
          <img
            src={facialExpressionMap[facialExpression] || ""}
            alt={facialExpression}
            style={{ maxHeight: 120 }}
          />
        </div>
      )}
      {movement && (
        <div>
          <div className="text-xs text-gray-500 text-center mb-1">Movement: {movement}</div>
          <img
            src={movementMap[movement] || ""}
            alt={movement}
            style={{ maxHeight: 120 }}
          />
        </div>
      )}
    </div>
  );
};

export default BehavioralDisplay;