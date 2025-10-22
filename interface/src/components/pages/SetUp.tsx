import { useState } from 'react';
import { useNavigate } from 'react-router-dom';

const SetUp = () => {
  const navigate = useNavigate();
  const [condition, setCondition] = useState('');
  const [participantId, setParticipantId] = useState('');

  const handleSave = () => {
    localStorage.setItem('condition', condition);
    localStorage.setItem('participantId', participantId);
    alert('Settings saved successfully!');
  };

  const handleClearSettings = () => {
    const confirmed = window.confirm('Are you sure you want to clear all settings?');
    if (confirmed) {
      localStorage.removeItem('condition');
      localStorage.removeItem('participantId');
      setCondition('');
      setParticipantId('');
      alert('Settings cleared.');
    }
  };

  const handleViewCurrentSettings = () => {
    const savedCondition = localStorage.getItem('condition');
    const savedId = localStorage.getItem('participantId');
    alert(
      `Current participant ID: ${savedId || 'None set'}\nCurrent condition: ${savedCondition || 'None set'}`
    );
  };

  const handleStartButton = () => {
    navigate('/prestudy');
  };

  return (
    <div className="content p-6 max-w-xl mx-auto">
      <h2 className="text-2xl font-bold mb-6">Set Up</h2>

      <div className="space-y-6">
        {/* Dropdown for condition selection */}
        <div>
          <label className="block mb-2 font-medium">Participant ID:</label>
          <input
            type="text"
            className="w-full px-4 py-2 border rounded-md"
            value={participantId}
            onChange={(e) => setParticipantId(e.target.value)}
            placeholder="Enter participant ID"
          />
        </div>

        <div>
          <label className="block mb-2 font-medium">Select Condition:</label>
          <select
            className="w-full px-4 py-2 border rounded-md"
            value={condition}
            onChange={(e) => setCondition(e.target.value)}
          >
            <option value="">-- Select Condition --</option>
            <option value="experimental">Experimental</option>
          </select>
        </div>

        {/* Action Buttons */}
        <div className="flex flex-col sm:flex-row gap-3 pt-4 mb-10">
          <button
            onClick={handleSave}
            disabled={!condition || !participantId}
            className={`flex-1 py-3 px-6 rounded-lg font-semibold transition-all ${
              condition && participantId
                ? 'bg-blue-500 hover:bg-blue-600 text-white shadow-md hover:shadow-lg'
                : 'bg-gray-300 text-gray-500 cursor-not-allowed'
            }`}
          >
            Save Settings
          </button>

          <button
            onClick={handleViewCurrentSettings}
            className="flex-1 py-3 px-6 bg-green-500 hover:bg-green-600 text-white rounded-lg font-semibold transition-all shadow-md hover:shadow-lg"
          >
            View Current Settings
          </button>

          <button
            onClick={handleClearSettings}
            className="flex-1 py-3 px-6 bg-red-500 hover:bg-red-600 text-white rounded-lg font-semibold transition-all shadow-md hover:shadow-lg"
          >
            Clear Settings
          </button>
        </div>
      </div>

      {/* Navigation */}
      <button
        onClick={handleStartButton}
        className="mt-4 w-full py-3 px-6 bg-purple-500 hover:bg-purple-600 text-white rounded-lg font-semibold transition-all shadow-md hover:shadow-lg"
      >
        Start Task Interface
      </button>
    </div>
  );
};

export default SetUp;