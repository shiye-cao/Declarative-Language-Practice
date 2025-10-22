import React, { useState } from 'react'
import { useNavigate } from 'react-router-dom'

interface QuestionnaireData {
  age: string;
  gender: string;
  native: string;
  proficiency: string;
  education: string;
  major: string;
  aiFamiliarity: string; // AI familiarity
  aiFrequency: string; // AI tool usage frequency
  vaFrequency: string; // Voice assistant usage frequency
  promptEngineeringExperience: string;
  promptEngineeringConfidence: string;
  promptEngineeringDescription: string;
}

const PreStudyQuestionnaire: React.FC = () => {
  const [form, setForm] = useState<QuestionnaireData>({
    age: '',
    gender: '',
    native: '',
    proficiency: '',
    education: '',
    major: '',
    aiFamiliarity: '',
    aiFrequency: '',
    vaFrequency: '',
    promptEngineeringExperience: '',
    promptEngineeringConfidence: '',
    promptEngineeringDescription: '',
  });

  const [submitting, setSubmitting] = useState(false)
  const [started, setStarted] = useState(false);
  const navigate = useNavigate()

  const handleChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>
  ) => {
    const { name, value } = e.target;
    setForm(f => ({ ...f, [name]: value }));
  };

  const submitPreStudy = async (payload: any) => {
    try {
      const response = await fetch('http://localhost:8000/prestudy', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      if (!response.ok) throw new Error('Submission failed');
      return await response.json();
    } catch (error) {
      console.error('Submission error:', error);
      return null;
    }
  };

  const handleSubmit = async (e: { preventDefault: () => void }) => {
    e.preventDefault();
    setSubmitting(true);

    const participantId = localStorage.getItem('participantId');
    const submissionTime = new Date().toISOString();
    const condition = localStorage.getItem('condition') ?? '';
    const startTime = localStorage.getItem('preStudyQuestionnaire_landingTime') || submissionTime;
    const timeTaken = ((new Date(submissionTime).getTime() - new Date(startTime).getTime()) / 1000).toString();


    // Map form fields to backend-required keys
    const payload = {
      startTime,
      submissionTime,
      timeTaken,
      participantId,
      condition,
      age: form.age,
      gender: form.gender,
      education: form.education,
      major: form.major,
      aiFamiliarity: form.aiFamiliarity,
      aiFrequency: form.aiFrequency,
      vaFrequency: form.vaFrequency,
      native: form.native,
      proficiency: form.proficiency,
      promptEngineeringExperience: form.promptEngineeringExperience,
      promptEngineeringConfidence: form.promptEngineeringConfidence,
      promptEngineeringDescription: form.promptEngineeringDescription,
    };

    localStorage.setItem('preStudyQuestionnaire', JSON.stringify(payload));
    console.log('Submitting payload:', payload);
    const submitResult = await submitPreStudy(payload);
    if (!submitResult || submitResult.status !== 'success') {
        alert('Submission failed, please try again.');
    } else {
        alert('Submitted!');
        navigate('/intro');
    }
    setSubmitting(false);
  };

  if (!started) {
    return (
      <div className="max-w-2xl mx-auto p-10 flex flex-col items-center justify-center">
        <h2 className="text-2xl font-bold mb-8">Pre‑Study Questionnaire</h2>
        <button
          className="px-8 py-4 bg-blue-600 text-white rounded text-lg hover:bg-blue-700"
          onClick={() => {
            setStarted(true);
            localStorage.setItem('preStudyQuestionnaire_landingTime', new Date().toISOString());
          }}
        >
          Start Pre-study Questionnaire
        </button>
      </div>
    );
  }

  return (
    <div className="max-w-4xl mx-auto p-6">
      <h2 className="text-2xl font-bold mb-6">Pre‑Study Questionnaire</h2>
      <form onSubmit={handleSubmit} className="space-y-12 text-left">
        {/* Gender */}
        <div>
          <label className="block font-medium mb-1 text-lg">Gender <span className="text-red-500">*</span></label>
          <select
            name="gender"
            required
            value={form.gender}
            onChange={handleChange}
            className="w-full border rounded p-2"
          >
            <option value="" disabled>Select…</option>
            <option>Female</option>
            <option>Male</option>
            <option>Non‑binary</option>
            <option>Prefer not to say</option>
          </select>
        </div>

        {/* Age */}
        <div>
          <label className="block text-lg font-medium mb-1">Age <span className="text-red-500">*</span></label>
          <input
            type="number"
            name="age"
            required
            value={form.age}
            onChange={handleChange}
            className="w-full border rounded p-2"
          />
        </div>

        {/* Highest degree (radio group) */}
        <fieldset className="space-y-4">
          <legend className="font-medium mb-1 text-lg">
            Highest level of school completed or highest degree earned <span className="text-red-500">*</span>
          </legend>
          {[
            'High school or equivalent',
            "Associate's degree",
            "Bachelor's degree",
            "Master's degree",
            'Doctoral degree',
            'Other',
          ].map(option => (
            <label key={option} className="flex items-center space-x-3 cursor-pointer">
              <input
                type="radio"
                name="education"
                value={option}
                checked={form.education === option}
                onChange={handleChange}
                required
                className="form-radio h-5 w-5 text-blue-600"
              />
              <span className="text-gray-800">{option}</span>
            </label>
          ))}
        </fieldset>

        {/* Native Language */}
        <fieldset className="space-y-4">
          <legend className="font-medium mb-1 text-lg">
          What is your native language (the first language you learned at home)?<span className="text-red-500">*</span>
          </legend>
          <input
            type="text"
            name="native"
            required
            value={form.native}
            onChange={handleChange}
            className="w-full border rounded p-2"
          />
        </fieldset>

        {/* Language Proficiency */}
        <fieldset className="space-y-4">
          <legend className="font-medium mb-1 text-lg">
            What is your English proficiency level? <span className="text-red-500">*</span>
          </legend>
          <select
            name="proficiency"
            required
            value={form.proficiency}
            onChange={handleChange}
            className="w-full border rounded p-2"
          >
            <option value="" disabled>Select…</option>
            <option>Beginner</option>
            <option>Intermediate</option>
            <option>Fluent</option>
            <option>Native</option>
          </select>
        </fieldset>

        {/* Major / Field*/}
        <fieldset className="space-y-4">
          <label className="font-medium mb-1 text-lg">
            Current or most recent major/field of study <span className="text-red-500">*</span>
          </label>
            <input
            type="text"
            name="major"
            required
            value={form.major}
            onChange={handleChange}
            className="w-full border rounded p-2"
          />
        </fieldset>

        {/* AI Familiarity */}
        <fieldset className="space-y-3">
          <legend className="text-lg font-medium">
            How familiar are you with artificial intelligence (AI) in general? <span className="text-red-500">*</span>
          </legend>
          {[
            "Not at all familiar",
            "Slightly familiar",
            "Moderately familiar",
            "Very familiar",
            "Extremely familiar"
          ].map(option => (
            <label key={option} className="flex items-center space-x-3 cursor-pointer">
              <input
                type="radio"
                name="aiFamiliarity"
                value={option}
                checked={form.aiFamiliarity === option}
                onChange={handleChange}
                required
                className="form-radio h-5 w-5 text-blue-600"
              />
              <span className="text-gray-800">{option}</span>
            </label>
          ))}
        </fieldset>

        {/* AI Tool Usage Frequency */}
        <fieldset className="space-y-3">
          <legend className="text-lg font-medium">
            How frequently do you use text-based AI tools (e.g., OpenAI ChatGPT, Claude, Gemini, etc.)? <span className="text-red-500">*</span>
          </legend>
          {[
            "Never",
            "Less than once a month",
            "A few times a month",
            "A few times a week",
            "Daily or almost daily"
          ].map(option => (
            <label key={option} className="flex items-center space-x-3 cursor-pointer">
              <input
                type="radio"
                name="aiFrequency"
                value={option}
                checked={form.aiFrequency === option}
                onChange={handleChange}
                required
                className="form-radio h-5 w-5 text-blue-600"
              />
              <span className="text-gray-800">{option}</span>
            </label>
          ))}
        </fieldset>

        {/* VA Usage Frequency */}
        <fieldset className="space-y-3">
          <legend className="text-lg font-medium">
            How frequently do you use voice-based AI tools (e.g., Amazon Alexa, Amazon Echo Dot, Google Assistant, Apple Siri, Microsoft Cortana, Samsung Bixby, or ChatGPT voice assistant)? <span className="text-red-500">*</span>
          </legend>
          {[
            "Never",
            "Less than once a month",
            "A few times a month",
            "A few times a week",
            "Daily or almost daily"
          ].map(option => (
            <label key={option} className="flex items-center space-x-3 cursor-pointer">
              <input
                type="radio"
                name="vaFrequency"
                value={option}
                checked={form.vaFrequency === option}
                onChange={handleChange}
                required
                className="form-radio h-5 w-5 text-blue-600"
              />
              <span className="text-gray-800">{option}</span>
            </label>
          ))}
        </fieldset>

        {/* Prompt Engineering Experience */}
        <fieldset className="space-y-3">
          <legend className="text-lg font-medium">
            How much experience do you have with prompt engineering (systematically designing and refining prompts to guide AI outputs)? <span className="text-red-500">*</span>
          </legend>
          {[
            "Never used - I have no experience with prompt engineering.",
            "Rarely used - I’ve tried it a few times with limited exposure.",
            "Occasionally used - I use it sometimes when needed but not regularly.",
            "Frequently used - I apply it often in my work or projects.",
            "Very frequently used - I use it consistently and it’s a regular part of my work."
          ].map(option => (
            <label key={option} className="flex items-center space-x-3 cursor-pointer">
              <input
                type="radio"
                name="promptEngineeringExperience"
                value={option}
                checked={form.promptEngineeringExperience === option}
                onChange={handleChange}
                required
                className="form-radio h-5 w-5 text-blue-600"
              />
              <span className="text-gray-800">{option}</span>
            </label>
          ))}
        </fieldset>

        {/* Prompt Engineering Confidence */}
        <fieldset className="space-y-3">
          <legend className="text-lg font-medium">
            How confident are you in your ability in prompt engineering? <span className="text-red-500">*</span>
          </legend>
          {[
            "Very not confident",
            "Not confident",
            "Neither",
            "Confident",
            "Very confident"
          ].map(option => (
            <label key={option} className="flex items-center space-x-3 cursor-pointer">
              <input
                type="radio"
                name="promptEngineeringConfidence"
                value={option}
                checked={form.promptEngineeringConfidence === option}
                onChange={handleChange}
                required
                className="form-radio h-5 w-5 text-blue-600"
              />
              <span className="text-gray-800">{option}</span>
            </label>
          ))}
        </fieldset>

        {/* Prompt Engineering Description */}
        <fieldset className="space-y-4">
          <label className="font-medium mb-1 text-lg">
            Please describe your experience with prompt engineering. When did you use it, and for what purposes? <span className="text-red-500">*</span>
          </label>
          <textarea
            name="promptEngineeringDescription"
            required
            value={form.promptEngineeringDescription || ''}
            onChange={handleChange}
            className="w-full border rounded p-2 min-h-[80px]"
            placeholder="Describe your experience..."
          />
        </fieldset>
        
        <button
          type="submit"
          disabled={submitting}
          className="w-full bg-blue-600 text-white rounded py-2 hover:bg-blue-700 disabled:opacity-50"
        >
          {submitting ? 'Submitting…' : 'Start Study'}
        </button>
      </form>
    </div>
  )
}

export default PreStudyQuestionnaire;