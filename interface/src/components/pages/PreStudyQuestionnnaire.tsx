import React from 'react'
import { useNavigate } from 'react-router-dom'

const PreStudyQuestionnnaire: React.FC = () => {
  const navigate = useNavigate()
  return (
    <div className="max-w-2xl mx-auto p-10 flex flex-col items-center justify-center">
      <h2 className="text-2xl font-bold mb-6">Pre‑Study (Placeholder)</h2>
      <div className="flex space-x-4">
        <button
          className="px-6 py-3 bg-blue-600 text-white rounded hover:bg-blue-700"
          onClick={() => navigate('/interface')}
        >
          Go to Interface
        </button>
        <button
          className="px-6 py-3 bg-gray-600 text-white rounded hover:bg-gray-700"
          onClick={() => navigate('/intro')}
        >
          Go to Task Intro
        </button>
      </div>
    </div>
  )
}

export default PreStudyQuestionnnaire;