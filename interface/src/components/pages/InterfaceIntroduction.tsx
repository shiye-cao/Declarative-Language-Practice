import { useNavigate } from "react-router-dom";

const InterfaceIntroduction = () => {
  const navigate = useNavigate();

  const handleNavigate = () => {
    navigate("/intro");
  };

  return (
    <div className="content">
      <h2 className="mx-auto text-2xl font-bold mb-6">Interface Introduction</h2>
      <div className="mx-auto ml-20">
        <p>You will first draft an initial prompt and then this interface shown below will help you iteratively test and improve the prompt.</p>
        <img
          src="/interface.png"
          alt="Interface Example"
          className="w-full max-w-4xl mt-5 mx-auto"
        />

        <p>Please click “next” to proceed once you have read through the interface.</p>
        <br />
        <br />
      </div>
      <button className="py-3 px-10 mx-auto block" onClick={handleNavigate}>Next</button>
    </div>
  );
};

export default InterfaceIntroduction;