// import React from 'react'
// import { useNavigate } from 'react-router-dom'

// const SetUp: React.FC = () => {
//   const navigate = useNavigate()
//   return (
//     <div className="content p-6 max-w-xl mx-auto text-center">
//       <h1 className="text-3xl font-bold mb-8">Declarative Language Learning</h1>
//       <button
//         onClick={() => navigate('/prestudy')}
//         className="px-6 py-3 bg-blue-600 text-white rounded-lg hover:bg-blue-700"
//       >
//         Start
//       </button>
//     </div>
//   )
// }

// export default SetUp;

import React from 'react'
import { useNavigate } from 'react-router-dom'

const SetUp: React.FC = () => {
  const navigate = useNavigate()

  return (
    <div className="relative h-screen flex flex-col justify-center items-center text-center bg-white">

      <h1 className="text-3xl font-bold mb-6 line-through text-gray-500">
        Start learning today.
      </h1>
      <h2 className="text-2xl mb-8 text-gray-800">
        We have time to learn declarative language today.
      </h2>

      <button
        onClick={() => navigate('/prestudy')}
        className="px-6 py-3 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition duration-200"
      >
        Start
      </button>
    </div>
  )
}

export default SetUp
