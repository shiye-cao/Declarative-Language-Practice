import React from 'react'
import { useNavigate } from 'react-router-dom'

const SetUp: React.FC = () => {
  const navigate = useNavigate()
  return (
    <div className="content p-6 max-w-xl mx-auto text-center">
      <h1 className="text-3xl font-bold mb-8">Declarative Language Learning</h1>
      <button
        onClick={() => navigate('/prestudy')}
        className="px-6 py-3 bg-blue-600 text-white rounded-lg hover:bg-blue-700"
      >
        Start
      </button>
    </div>
  )
}

export default SetUp;