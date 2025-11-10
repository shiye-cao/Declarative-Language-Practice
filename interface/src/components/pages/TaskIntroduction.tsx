import React from 'react'
import { useNavigate } from 'react-router-dom'

const RobotInteraction: React.FC = () => {
  const navigate = useNavigate()

  return (
    <div className="min-h-screen p-10 bg-white max-w-5xl mx-auto flex flex-col">
      {/* Setup Instructions */}
      <section className="mb-10">
        <h1 className="text-3xl font-bold mb-4">Set Up Your Robot</h1>
        <p className="text-lg text-gray-700 mb-2">
          Welcome! Before we start, make sure your robot is ready to respond to your declarative language. 
        </p>
        <ul className="list-disc list-inside text-gray-700 space-y-1">
          <li>Ensure your microphone is on and working.</li>
          <li>Speak clearly in short, declarative sentences.</li>
          <li>Watch the robot respond to your language in real-time.</li>
        </ul>
      </section>

      {/* Main Interaction Area */}
      <section className="flex flex-col md:flex-row mb-10 gap-8">
        {/* Left: Audio placeholder */}
        <div className="flex-1 flex justify-center items-center border border-gray-300 rounded-lg p-6 bg-gray-50">
          <img
            src="https://cdn-icons-png.flaticon.com/512/727/727240.png"
            alt="Voice/audio placeholder"
            className="w-32 h-32 opacity-80"
          />
        </div>

        {/* Right: Feedback area */}
        <div className="flex-1 border border-gray-300 rounded-lg p-6 bg-gray-50">
          <h2 className="text-xl font-semibold mb-4">Feedback</h2>
          <p className="text-gray-700">
            Your robot will give feedback here based on the declarative sentences you speak. 
            For now, this is a placeholder area.
          </p>
        </div>
      </section>

      {/* Bottom: Done Learning Button */}
      <div className="flex justify-center mt-auto">
        <button
          onClick={() => navigate('/feedback')}
          className="px-6 py-3 bg-blue-600 text-white rounded hover:bg-blue-700 transition"
        >
          Done Learning
        </button>
      </div>
    </div>
  )
}

export default RobotInteraction
