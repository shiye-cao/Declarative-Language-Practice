import { useState, useRef, useEffect } from 'react';

type AttributeCardProps = {
  name: string;
  description: string;
  value: string | number;
  details: string;
};

function AttributeCard({ name, description, value, details }: AttributeCardProps) {
  const [isHovered, setIsHovered] = useState(false);
  const [tooltipPosition, setTooltipPosition] = useState({ isLeft: false });
  const cardRef = useRef<HTMLDivElement>(null);
  const infoButtonRef = useRef<HTMLDivElement>(null);

  // Determine tooltip position based on available space
  useEffect(() => {
    if (isHovered && cardRef.current && infoButtonRef.current) {
      const cardRect = cardRef.current.getBoundingClientRect();
      const rightEdgeDistance = window.innerWidth - cardRect.right;
      
      // If less than 200px from right edge, position tooltip to the left
      setTooltipPosition({ isLeft: rightEdgeDistance < 200 });
    }
  }, [isHovered]);

  return (
    <div 
      ref={cardRef}
      className="relative flex justify-between items-stretch border rounded-xl shadow bg-white w-full max-w-md"
    >
      <div className="flex flex-col border-r p-4 h-full w-3/4">
        <p className="text-lg font-semibold text-gray-800">{name}</p>
        <p className="text-sm text-gray-500">{description}</p>
      </div>
      <div className="flex items-center px-3">
        <p className="text-blue-900 font-bold whitespace-nowrap">{value}</p>
      </div>
      <div 
        ref={infoButtonRef}
        className="bg-red-400 rounded-full w-5 h-5 mt-2 mr-2 flex items-center justify-center cursor-help"
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
      >
      </div>
      
      {isHovered && (
        <div 
          className={`absolute z-10 bg-white border shadow-lg rounded-lg p-3 w-64 text-sm text-gray-700 whitespace-pre-line ${
            tooltipPosition.isLeft 
              ? "right-10px mr-2 top-0" 
              : "left-1/2 -translate-x-1/2 -top-0"
          }`}
        >
          <p>{details}</p>
        </div>
      )}
    </div>
  );
}

export default AttributeCard;