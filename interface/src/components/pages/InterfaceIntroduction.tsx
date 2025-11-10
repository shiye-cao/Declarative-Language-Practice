import React from 'react'
import { useNavigate } from 'react-router-dom'

const DeclarativeLanguageBasics: React.FC = () => {
  const navigate = useNavigate()

  return (
    <div className="min-h-screen p-10 bg-white max-w-4xl mx-auto flex flex-col">
      {/* Page Title */}
      <h1 className="text-4xl font-bold text-center mb-12">
        Declarative Language Basics
      </h1>

      {/* Section 1: What is declarative language */}
      <section className="mb-12">
        <h2 className="text-2xl font-semibold mb-4">
          What is declarative language and why is it helpful?
        </h2>
        <p className="text-lg text-gray-700">
          Declarative language is a way of expressing what you want in clear, simple sentences. 
          Instead of giving direct commands like "Do this," declarative sentences describe 
          your intentions. This helps children understand expectations, reduces frustration, 
          and promotes communication skills.
        </p>
      </section>

      {/* Section 2: Examples */}
      <section className="mb-12">
        <h2 className="text-2xl font-semibold mb-4">
          Examples: Direct commands vs. Declarative language
        </h2>
        <table className="w-full border-collapse text-left">
          <thead>
            <tr>
              <th className="border-b px-4 py-2">Direct Command</th>
              <th className="border-b px-4 py-2">Declarative Language</th>
            </tr>
          </thead>
          <tbody>
            <tr className="bg-gray-50">
              <td className="border px-4 py-2">“Put your toys away.”</td>
              <td className="border px-4 py-2">“It’s time for the toys to go back on the shelf.”</td>
            </tr>
            <tr>
              <td className="border px-4 py-2">“Sit down!”</td>
              <td className="border px-4 py-2">“We are going to sit now for story time.”</td>
            </tr>
            <tr className="bg-gray-50">
              <td className="border px-4 py-2">“Stop yelling.”</td>
              <td className="border px-4 py-2">“We use quiet voices inside the classroom.”</td>
            </tr>
          </tbody>
        </table>
      </section>

      {/* Section 3: Key / useful words and phrases */}
      <section className="mb-12">
        <h2 className="text-2xl font-semibold mb-4">
          Key / Useful Words and Phrases
        </h2>
        <ul className="list-disc list-inside text-lg text-gray-700 space-y-2">
          <li>“I notice that…”</li>
          <li>“It looks like…”</li>
          <li>“We are going to…”</li>
          <li>“It’s time to…”</li>
          <li>“You can try…”</li>
          <li>“Let’s see what happens if…”</li>
        </ul>
      </section>

      {/* Back button at the bottom */}
      <div className="flex justify-center mt-auto">
        <button
          onClick={() => navigate('/prestudy')}
          className="px-6 py-3 bg-gray-300 text-gray-800 rounded hover:bg-gray-400 transition"
        >
          &larr; Back
        </button>
      </div>
    </div>
  )
}

export default DeclarativeLanguageBasics
