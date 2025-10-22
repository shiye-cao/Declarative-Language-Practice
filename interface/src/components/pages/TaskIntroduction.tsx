import { useNavigate } from "react-router-dom";

const TaskIntroduction = () => {
  const navigate = useNavigate();

  const handleNavigate = () => {
    const savedCondition = localStorage.getItem('condition');
    console.log('Saved condition:', savedCondition);
    if (savedCondition === "experimental") {
      navigate("/agent");
    } else {
      navigate(`/${savedCondition}`);
    }
  };

  return (
    <div className="content">
      <div className="max-w-6xl mx-auto text-lg">
        <h2 className="text-2xl font-bold mb-6">Design Instructions</h2>
        <p className="mb-10"><strong>ACE</strong> is an interface designed to help you design effective human-robot conversations. ACE does not have any prior knowledge of what you are designing. So, you will first have a discussion with ACE to provide ACE with more context about the task and discuss the robot's expected role and any desired behavior guidelines for the robot. Through discussion, ACE will help you draft an initial instruction prompt for the robot. This instruction prompt shapes how Luna behaves.</p>

        <div className="flex justify-center w-full">
          <img
            src="/instruction-images/ace-interface.png"
            alt="ACE intial prompting interface"
            className="w-250 mx-auto mb-10 border border-gray-400 rounded-lg shadow-sm"
          />
        </div>

        <p className="mb-10">After drafting the initial instruction prompt, put yourself into a child’s shoes and test out Luna to see how it performs on the task. </p>

        <div className="flex justify-center w-full">
          <img
            src="/instruction-images/prompt-panel.png"
            alt="Luna prompt panel"
            className="w-250 mx-auto mb-10 border border-gray-400 rounded-lg shadow-sm"
          />
        </div>

        <p className="mb-10">After testing, read through the transcript. Annotate the transcript content by dragging your mouse over the text to label what you liked and disliked, add tags, and provide comments on the content of the conversation. The annotations can serve as useful feedback to help you improve your prompt.</p>

        <div className="flex justify-center w-full">
          <img
            src="/instruction-images/transcript-panel.png"
            alt="Luna transcript panel"
            className="w-250 mx-auto mb-10 border border-gray-400 rounded-lg shadow-sm"
          />
        </div>

        <p className="mb-5">ACE can also process your annotations and help summarize and translate your feedback into recommended prompt adjustments. You can also edit ACE’s suggestions. After that, ACE can also help you update your prompt based on the annotation summary. Feel free to make edits to the prompt and use the “Save Suggested Prompt” button to move the prompt to the left for it to be used by Luna.</p>

        <div className="flex justify-center w-full">
          <img
            src="/instruction-images/feedback-panel.png"
            alt="Luna transcript panel"
            className="w-250 mx-auto mb-10 border border-gray-400 rounded-lg shadow-sm"
          />
        </div>

        <p className="mb-5"><strong>You can repeat this “test and refine” process as many times as you want until you feel that Luna is ready to use for the task!</strong></p>
        <p className="mb-5">You can also view previous iterations of your prompt and transcription history using the “View Prompt & Transcript History” button on the bottom right corner. If you’re ever unsure what to do next, check the instructions at the top of the page for guidance on your next step.</p>
        <br />
      </div>
      <button className="py-3 px-10 mx-auto block" onClick={handleNavigate}>Next</button>
    </div>
  );
};

export default TaskIntroduction;