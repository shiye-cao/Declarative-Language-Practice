import React, { useEffect, useState, useRef, useMemo, useCallback } from "react";
import { useParams } from "react-router-dom";


interface Highlight {
 id: string;
 text: string;
 startIndex: number;
 endIndex: number;
 style: 'highlight' | 'strikethrough';
 reasoning: string;
}


interface HighlightPopupProps {
 selectedText: string;
 position: { x: number; y: number };
 onHighlight: (style: 'highlight' | 'strikethrough', reasoning: string) => void;
 onCancel: () => void;
 overlappingHighlights?: Highlight[];
 onReplaceHighlights?: (highlightIds: string[], style: 'highlight' | 'strikethrough', reasoning: string) => void;
}


const HighlightPopup: React.FC<HighlightPopupProps> = ({
 selectedText,
 onHighlight,
 onCancel,
 overlappingHighlights = [],
 onReplaceHighlights
}) => {
 const [reasoning, setReasoning] = useState("");
 const [selectedStyle, setSelectedStyle] = useState<'highlight' | 'strikethrough' | null>(null);
 const [replaceMode, setReplaceMode] = useState(false);
 const [selectedTags, setSelectedTags] = useState<string[]>([]);


 const tags = ["Necessary", "Unnecessary", "Clear", "Ambiguous", "Informative", "Redundant", "Concise", "Wordy", "On-topic", "Off-topic", "Helpful", "Confusing"]; // Some tags


 const handleTagToggle = (tag: string) => {
   setSelectedTags(prev =>
     prev.includes(tag) ? prev.filter(t => t !== tag) : [...prev, tag]
   );
 };


 const handleSubmit = () => {
   if (!selectedStyle) return;
   const reasoningText = reasoning.trim();
   let reasoningWithTags = "";
   if (selectedTags.length > 0) {
     reasoningWithTags = selectedTags.join(", ") + (reasoningText ? `: ${reasoningText}` : "");
   } else {
     reasoningWithTags = reasoningText;
   }
   if (replaceMode && overlappingHighlights.length > 0 && onReplaceHighlights) {
     onReplaceHighlights(overlappingHighlights.map(h => h.id), selectedStyle, reasoningWithTags);
   } else {
     onHighlight(selectedStyle, reasoningWithTags);
   }
   setReasoning("");
   setSelectedTags([]);
   setSelectedStyle(null);
 };


 const hasOverlaps = overlappingHighlights.length > 0;


 return (
   <div
     className="fixed bg-white border-2 border-gray-300 rounded-lg shadow-lg z-50 w-80 flex flex-col"
     style={{
       left: window.innerWidth * 0.625,
       top: window.innerHeight * 0.10,
       width: 450,
       height: 550,
     }}
   >
     {/* Scrollable content area */}
     <div className="overflow-y-auto flex-1 p-4">
       <div className="mb-3">
         <p className="text-sm font-semibold mb-2">Selected text:</p>
         <p className="text-sm bg-gray-100 p-2 rounded italic">"{selectedText}"</p>
       </div>


       {hasOverlaps && (
         <div className="mb-3 p-2 bg-orange-50 border border-orange-200 rounded">
           <p className="text-sm font-semibold text-orange-700 mb-1">
             This selection overlaps with {overlappingHighlights.length} existing highlight{overlappingHighlights.length > 1 ? 's' : ''}:
           </p>
           {overlappingHighlights.map(highlight => (
             <div key={highlight.id} className="text-xs text-orange-600 mb-1">
               • <span className={highlight.style === 'highlight' ? 'bg-yellow-200 px-1 rounded' : 'line-through'}>
                 "{highlight.text.substring(0, 30)}{highlight.text.length > 30 ? '...' : ''}"
               </span>
             </div>
           ))}
           <div className="mt-2">
             <label className="flex items-center text-sm">
               <input
                 type="checkbox"
                 checked={replaceMode}
                 onChange={(e) => setReplaceMode(e.target.checked)}
                 className="mr-2"
               />
               Replace existing highlights with new one
             </label>
           </div>
         </div>
       )}
      
       <div className="mb-3">
         <p className="text-sm font-semibold mb-2">Choose:</p>
         <div className="flex gap-2">
           <button
             className={`px-3 py-1 rounded border-2 ${selectedStyle === 'highlight' ? 'bg-yellow-400 border-yellow-500' : 'bg-yellow-100 border-yellow-300'}`}
             onClick={() => setSelectedStyle('highlight')}
             type="button"
           >
             <span className="bg-yellow-200 px-1 rounded">
               Liked
             </span>
           </button>
           <button
             className={`px-3 py-1 rounded border-2 ${selectedStyle === 'strikethrough' ? 'bg-red-400 border-red-500' : 'bg-red-100 border-red-300'}`}
             onClick={() => setSelectedStyle('strikethrough')}
             type="button"
           >
            <span style={{ textDecoration: 'line-through', textDecorationColor: 'red', color: 'black', textDecorationThickness: '0.2em' }}>
               Disliked
             </span>
           </button>
         </div>
       </div>
      
       <div className="mb-3">
         <p className="text-sm font-semibold mb-2">Select tags for reasoning (optional):</p>
         <div className="flex gap-2 flex-wrap">
           {tags.map(tag => (
             <button
               key={tag}
               type="button"
               className={`px-3 py-1 rounded border-2 ${selectedTags.includes(tag) ? 'bg-blue-200 border-blue-500' : 'bg-gray-100 border-gray-300'}`}
               onClick={() => handleTagToggle(tag)}
             >
               {tag}
             </button>
           ))}
         </div>
       </div>
       <div className="mb-3">
         <label className="text-sm font-semibold mb-1 block">Please tell us (optional):</label>
         <textarea
           className="w-full p-2 border border-gray-300 rounded text-sm h-40 resize-none"
           value={reasoning}
           onChange={(e) => setReasoning(e.target.value)}
           placeholder="Why are you marking this text? (optional)"
         />
       </div>
     </div>
     <div className="flex justify-end gap-2 p-4 pt-2">
       <button
         className="px-3 py-1 bg-gray-200 rounded text-sm"
         onClick={onCancel}
       >
         Cancel
       </button>
       <button
         className="px-3 py-1 bg-blue-500 text-white rounded text-sm disabled:bg-gray-400"
         onClick={handleSubmit}
         disabled={!selectedStyle}
       >
         {replaceMode ? 'Replace' : 'Apply'}
       </button>
     </div>
   </div>
 );
};






interface TranscriptAnalysisProps {
 updateProgress: React.Dispatch<React.SetStateAction<any>>;
 progress: any;
 setSuggestions: React.Dispatch<React.SetStateAction<string[]>>;
 clearHighlightsTrigger?: number;
 transcript: string;
 // onBehaviorClick?: (behavior: { facialExpression?: string; movement?: string }) => void;
}




const TranscriptAnalysis = ({
 updateProgress,
 progress,
 setSuggestions,
 clearHighlightsTrigger,
 transcript,
 // onBehaviorClick,
}: TranscriptAnalysisProps) => {
 const [highlights, setHighlights] = useState<Highlight[]>([]);
 const [showPopup, setShowPopup] = useState(false);
 const [isGenerating, setIsGenerating] = useState(false);
 const [popupData, setPopupData] = useState<{
   text: string;
   startIndex: number;
   endIndex: number;
   position: { x: number; y: number };
 } | null>(null);


 const displayRef = useRef<HTMLDivElement>(null);


 const { idx } = useParams();
 const currentTaskIndex = parseInt(idx || "0");


 const taskOrder: number[] = JSON.parse(localStorage.getItem("taskOrder") || "[]");
 const studentID = taskOrder[currentTaskIndex];


 const originalTextRef = useRef("");
 const [displayText, setDisplayText] = useState(transcript || "");


 // Keep displayText in sync with transcript prop
 useEffect(() => {
   setDisplayText(transcript || "");
   originalTextRef.current = transcript || "";
 }, [transcript]);


 const updateHighlightsStorage = useCallback((newHighlights: Highlight[]) => {
   const storageKey = `${studentID}_highlights`;
   localStorage.setItem(storageKey, JSON.stringify(newHighlights));
 }, [studentID]);


  const clearAllHighlights = useCallback(() => {
   const confirmed = window.confirm("Are you sure you want to clear all annotations?");
   if (confirmed) {
     setHighlights([]);
     const storageKey = `${studentID}_highlights`;
     localStorage.removeItem(storageKey);
   }
 }, [studentID]);


 const findOverlappingHighlights = useCallback((startIndex: number, endIndex: number): Highlight[] => {
   return highlights.filter(highlight =>
     (startIndex < highlight.endIndex && endIndex > highlight.startIndex)
   );
 }, [highlights]);


 const exportToJSON = useCallback((): Promise<void> => {
   // Helper to extract tags and comment from reasoning string
   function parseReasoning(reasoning: string) {
     // Reasoning format: "tag1, tag2: comment" or "tag1, tag2"
     const [tagsPart, ...commentParts] = reasoning.split(":");
     const tags = tagsPart ? tagsPart.split(",").map(t => t.trim()).filter(Boolean) : [];
     const comment = commentParts.join(":").trim();
     return { tags, comment };
   }


   // Format each highlight for export
   function formatHighlight(h: Highlight) {
     const { tags, comment } = parseReasoning(h.reasoning);
     let inside = [];
     if (tags.length > 0) inside.push(`tag: ${tags.join(", ")}`);
     if (comment) inside.push(`comment: ${comment}`);
     if (inside.length > 0) {
       return `${h.text} (${inside.join(", ")})`;
     } else {
       return h.text;
     }
   }


   const liked = highlights.filter(h => h.style === 'highlight').map(formatHighlight);
   const disliked = highlights.filter(h => h.style === 'strikethrough').map(formatHighlight);


   const exportData = {
     "full-transcript": originalTextRef.current,
     "user-liked": liked,
     "user-disliked": disliked
   };
   // add timestamp to file name
   const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
   const outputFileName = `analysis_results_${timestamp}.txt`;


   const payload = {
     filename: outputFileName,
     content: JSON.stringify(exportData, null, 2)
   };


   return fetch("http://localhost:8000/save-analysis", {
     method: "POST",
     headers: { "Content-Type": "application/json" },
     body: JSON.stringify(payload)
   })
     .then(res => res.json())
     .then(data => {
       console.log("Saved:", data);
       alert("Analysis saved to server");
     })
     .catch(err => {
       console.error("Failed to save:", err);
       alert("Failed to save analysis");
     });
 }, [highlights]);


 const handleTextSelection = useCallback(() => {
   const selection = window.getSelection();
   if (!selection || selection.isCollapsed) return;


   const selectedText = selection.toString().trim();
   if (!selectedText) return;


   // Find the start index of the selected text in the displayText
   const container = displayRef.current;
   if (!container) return;


   // Get the plain text content of the transcript
   const transcriptText = displayText;


   // Find the first occurrence of the selected text in the transcript
   const startIndex = transcriptText.indexOf(selectedText);
   if (startIndex === -1) return; // Not found


   const endIndex = startIndex + selectedText.length;


   setPopupData({
     text: selectedText,
     startIndex,
     endIndex,
     position: { x: 0, y: 0 }, // You can improve this if you want to show the popup at the cursor
   });


   setShowPopup(true);


   // Optionally clear the selection
   // setTimeout(() => {
   //   const newSelection = window.getSelection();
   //   if (newSelection) {
   //     newSelection.removeAllRanges();
   //   }
   // }, 0);
 }, [displayText]);


 const handleHighlight = useCallback((style: 'highlight' | 'strikethrough', reasoning: string) => {
   if (!popupData) return;


   const newHighlight: Highlight = {
     id: Date.now().toString(),
     text: popupData.text,
     startIndex: popupData.startIndex,
     endIndex: popupData.endIndex,
     style,
     reasoning
   };


   const newHighlights = [...highlights, newHighlight];
   setHighlights(newHighlights);
   updateHighlightsStorage(newHighlights);
  
   setShowPopup(false);
   setPopupData(null);
 }, [popupData, highlights, updateHighlightsStorage]);


 const handleReplaceHighlights = useCallback((highlightIds: string[], style: 'highlight' | 'strikethrough', reasoning: string) => {
   if (!popupData) return;


   // Remove the overlapping highlights and add the new one
   const filteredHighlights = highlights.filter(h => !highlightIds.includes(h.id));
  
   const newHighlight: Highlight = {
     id: Date.now().toString(),
     text: popupData.text,
     startIndex: popupData.startIndex,
     endIndex: popupData.endIndex,
     style,
     reasoning
   };


   const newHighlights = [...filteredHighlights, newHighlight];
   setHighlights(newHighlights);
   updateHighlightsStorage(newHighlights);
  
   setShowPopup(false);
   setPopupData(null);
 }, [popupData, highlights, updateHighlightsStorage]);


 const handleDeleteHighlight = useCallback((highlightId: string) => {
   const newHighlights = highlights.filter(h => h.id !== highlightId);
   setHighlights(newHighlights);
   updateHighlightsStorage(newHighlights);
 }, [highlights, updateHighlightsStorage]);


 const handleCancelHighlight = useCallback(() => {
   setShowPopup(false);
   setPopupData(null);
 }, []);


 // Fixed rendering function using React elements instead of dangerouslySetInnerHTML
 const renderHighlightedText = useMemo(() => {
   if (highlights.length === 0) {
     return displayText.split('\n').map((line, i) => (
       <React.Fragment key={i}>
         {line}
         {i < displayText.split('\n').length - 1 && <br />}
       </React.Fragment>
     ));
   }


   // Sort highlights by start index
   const sortedHighlights = [...highlights].sort((a, b) => a.startIndex - b.startIndex);
      
   let currentIndex = 0;
   // Create segments
   const segments: Array<{
     text: string;
     highlights: Highlight[];
     startIndex: number;
     endIndex: number;
   }> = [];


   // Build segments properly
   let i = 0;
   while (i < displayText.length) {
     const activeHighlights = sortedHighlights.filter(h =>
       h.startIndex <= i && h.endIndex > i
     );


     if (activeHighlights.length > 0) {
       // Find the end of this highlighted section
       let endOfSection = i + 1;
       while (endOfSection < displayText.length) {
         const nextActiveHighlights = sortedHighlights.filter(h =>
           h.startIndex <= endOfSection && h.endIndex > endOfSection
         );
        
         if (nextActiveHighlights.length !== activeHighlights.length ||
             !nextActiveHighlights.every(h1 => activeHighlights.some(h2 => h2.id === h1.id))) {
           break;
         }
         endOfSection++;
       }


       // Add non-highlighted text before this section
       if (i > currentIndex) {
         segments.push({
           text: displayText.substring(currentIndex, i),
           highlights: [],
           startIndex: currentIndex,
           endIndex: i
         });
       }


       // Add highlighted section
       segments.push({
         text: displayText.substring(i, endOfSection),
         highlights: activeHighlights,
         startIndex: i,
         endIndex: endOfSection
       });


       currentIndex = endOfSection;
       i = endOfSection;
     } else {
       i++;
     }
   }


   // Add any remaining text
   if (currentIndex < displayText.length) {
     segments.push({
       text: displayText.substring(currentIndex),
       highlights: [],
       startIndex: currentIndex,
       endIndex: displayText.length
     });
   }


   // Convert segments to React elements
   return segments.map((segment, index) => {
     const text = segment.text;
    
     if (segment.highlights.length === 0) {
       return text.split('\n').map((line, lineIndex) => (
         <React.Fragment key={`${index}-${lineIndex}`}>
           {line}
           {lineIndex < text.split('\n').length - 1 && <br />}
         </React.Fragment>
       ));
     } else {
       // Apply highlight styles
       const styles = segment.highlights.map(h => h.style);
       const reasonings = segment.highlights.map(h => h.reasoning).join(' | ');
      
       const hasHighlight = styles.includes('highlight');
       const hasStrikethrough = styles.includes('strikethrough');
      
       let className = 'px-1 rounded';
       if (hasHighlight) className += ' bg-yellow-200';
       // if (hasStrikethrough) className += ' line-through text-red-500';
       // TODO
       if (hasStrikethrough) {
         return (
           <span
             key={index}
             className={className}
             title={reasonings}
             style={{
               textDecoration: 'line-through',
               textDecorationColor: 'red',
               textDecorationThickness: '0.2em',
               color: 'black'
             }}
           >
             {text.split('\n').map((line, lineIndex) => (
               <React.Fragment key={`${index}-${lineIndex}`}>
                 {line}
                 {lineIndex < text.split('\n').length - 1 && <br />}
               </React.Fragment>
             ))}
           </span>
         );
       }
       if (styles.length > 1) className += ' border-2 border-blue-400';
      
       return (
         <span key={index} className={className} title={reasonings}>
           {text.split('\n').map((line, lineIndex) => (
             <React.Fragment key={`${index}-${lineIndex}`}>
               {line}
               {lineIndex < text.split('\n').length - 1 && <br />}
             </React.Fragment>
           ))}
         </span>
       );
     }
   });
 }, [displayText, highlights]);


 const overlappingHighlights = useMemo(() => {
   if (!popupData) return [];
   return findOverlappingHighlights(popupData.startIndex, popupData.endIndex);
 }, [popupData, findOverlappingHighlights]);


 const handleClick = async () => {
   // Prepare annotation summary from highlights
   const highlightedTexts = highlights.filter(h => h.style === 'highlight').map(h => h.text);
   const strikethroughTexts = highlights.filter(h => h.style === 'strikethrough').map(h => h.text);


   const annotationSummary = `
     Liked: ${highlightedTexts.join("; ")}
     Unnecessary: ${strikethroughTexts.join("; ")}
   `;


   await exportToJSON();


   setIsGenerating(true);
   // Send to GPT backend
   try {
     const res = await fetch("http://localhost:8000/generate-feedback-summary", {
       method: "POST",
       headers: { "Content-Type": "application/json" },
       body: JSON.stringify({ userFeedback: annotationSummary}),
     });
     if (!res.ok) throw new Error("Failed to generate suggestions from annotation");
     const data = await res.json();
     // Set suggestions as an array (could split by sentences or just use as one suggestion)
     setSuggestions([data.aiPrompt]);
   } catch (err) {
     alert("Failed to process annotation with GPT.");
     setSuggestions(["[Error] Could not generate suggestions."]);
     console.error(err);
   } finally {
     setIsGenerating(false);
   }


 const pid = localStorage.getItem("participantId") || "";


   await fetch("http://localhost:8000/save-history", {
     method: "POST",
     headers: { "Content-Type": "application/json" },
     body: JSON.stringify({ participantId: pid }),
   });




   updateProgress((prev: any) => ({
     ...prev,
     feedbackProcessed: true,
   }));
 };
  // Load highlights from localStorage on component mount
 React.useEffect(() => {
   const storageKey = `${studentID}_highlights`;
   const savedHighlights = JSON.parse(localStorage.getItem(storageKey) || "[]");
   setHighlights(savedHighlights);
 }, [studentID]);


 useEffect(() => {
   if (clearHighlightsTrigger !== undefined) {
     setHighlights([]);
     // Optionally clear from localStorage as well
     const storageKey = `${studentID}_highlights`;
     localStorage.removeItem(storageKey);
   }
 }, [clearHighlightsTrigger, studentID]);


 useEffect(() => {
   if (progress.conversationStarted) {
     setDisplayText("");
     originalTextRef.current = "";
   }
 }, [progress.conversationStarted]);


 return (
   <>
     <div className="flex flex-col items-center justify-center">
       <div className={`w-full max-w-6xl h-125 ${highlights.length === 0 ? 'mb-8.5' : ''}`}>
         <label className="block mb-1 font-medium">Transcript</label>
         <div
           ref={displayRef}
           className="w-full h-full p-4 border border-gray-300 rounded font-mono overflow-auto whitespace-pre-wrap cursor-text select-text text-left text-xs"
           onMouseUp={handleTextSelection}
         >
           {displayText.trim().length === 0
             ? <span className="text-gray-400 italic">Your conversation transcript will appear here after you end conversation.</span>
             : renderHighlightedText}
         </div>
       </div>
      
       {highlights.length > 0 && (
         <div className="mt-4 w-full max-w-4xl mt-10">
           <div className="flex justify-between items-center mb-2">
             <h3 className="font-semibold text-sm">Your Annotations ({highlights.length}):</h3>
             <button
               onClick={clearAllHighlights}
               className="text-red-500 hover:text-red-700 text-xs px-3 py-1 border border-red-300 rounded hover:bg-red-50"
             >
               Delete All
             </button>
           </div>
           <div className="max-h-22 overflow-y-auto space-y-1 mb-2">
             {highlights.map((highlight) => (
               <div key={highlight.id} className="text-sm p-2 bg-gray-50 rounded flex justify-between items-start">
                 <div className="flex-1">
                   <span className={highlight.style === 'highlight' ? 'bg-yellow-200 px-1 rounded' : 'line-through'}>
                     "{highlight.text.substring(0, 30)}{highlight.text.length > 50 ? '...' : ''}"
                   </span>
                   <span className="ml-2 text-gray-600">- {highlight.reasoning}</span>
                 </div>
                 <button
                   onClick={() => handleDeleteHighlight(highlight.id)}
                   className="ml-2 text-red-500 hover:text-red-700 text-xs px-2 py-1 border border-red-300 rounded hover:bg-red-50"
                 >
                   Delete
                 </button>
               </div>
             ))}
           </div>
         </div>
       )}


       <div className="flex justify-center">
         <button
           className={`px-4 py-2 rounded bg-gray-200 hover:bg-gray-300 ${!progress.conversationEnded || isGenerating ? 'opacity-50 cursor-not-allowed' : ''}`}
           onClick={handleClick}
           disabled={!progress.conversationEnded || isGenerating}
         >
           {isGenerating ? "Processing..." : "Process Annotation"}
         </button>
       </div>
     </div>




     {showPopup && popupData && (
       <HighlightPopup
         selectedText={popupData.text}
         position={popupData.position}
         onHighlight={handleHighlight}
         onCancel={handleCancelHighlight}
         overlappingHighlights={overlappingHighlights}
         onReplaceHighlights={handleReplaceHighlights}
       />
     )}
   </>
 );
};


export default TranscriptAnalysis;

