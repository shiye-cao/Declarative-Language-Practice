import React from 'react'
import { useNavigate, useLocation } from 'react-router-dom'

interface FeedbackState {
  transcript?: { user: string; robot: string }[]
  notes?: string[]
}

const Feedback: React.FC = () => {
  const navigate = useNavigate()
  const location = useLocation()
  const state = location.state as FeedbackState | undefined

  // Example placeholders if no state is passed
  const transcript = state?.transcript || [
    { user: 'It’s time to clean up.', robot: 'Okay, the toys are being put away.' },
    { user: 'We are going to read a book.', robot: 'Great! Let’s start story time.' },
  ]

  const notes = state?.notes || [
    'Try to use more descriptive sentences.',
    'Focus on speaking slowly and clearly.',
    'Use key declarative phrases like "I notice that…" more often.',
  ]

  return (
    <div className="min-h-screen p-10 bg-white max-w-6xl mx-auto flex flex-col">
      <h1 className="text-3xl font-bold text-center mb-10">Feedback Summary</h1>

      <section className="flex flex-col md:flex-row gap-8 mb-10 flex-1">
        {/* Left column: Transcript */}
        <div className="flex-1 border border-gray-300 rounded-lg p-6 bg-gray-50">
          <h2 className="text-xl font-semibold mb-4">Transcript</h2>
          <div className="space-y-4 max-h-[400px] overflow-y-auto">
            {transcript.map((entry, index) => (
              <div key={index} className="space-y-1">
                <p className="font-semibold text-blue-700">User: {entry.user}</p>
                <p className="font-semibold text-green-700">Robot: {entry.robot}</p>
              </div>
            ))}
          </div>
        </div>

        {/* Right column: Notes for improvement */}
        <div className="flex-1 border border-gray-300 rounded-lg p-6 bg-gray-50">
          <h2 className="text-xl font-semibold mb-4">Areas for Improvement</h2>
          <ul className="list-disc list-inside space-y-2 text-gray-700 max-h-[400px] overflow-y-auto">
            {notes.map((note, index) => (
              <li key={index}>{note}</li>
            ))}
          </ul>
        </div>
      </section>

      {/* Bottom: Back to Home button */}
      <div className="flex justify-center mt-auto">
        <button
          onClick={() => navigate('/prestudy')}
          className="px-6 py-3 bg-blue-600 text-white rounded hover:bg-blue-700 transition"
        >
          Back to Home
        </button>
      </div>
    </div>
  )
}

export default Feedback
