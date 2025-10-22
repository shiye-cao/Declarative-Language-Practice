import React, { useState } from 'react';

interface QuestionnaireData {
  first: string;
  last: string;
  email: string;
  confirmEmail: string;
}

const Compensation: React.FC = () => {
  const [form, setForm] = useState<QuestionnaireData>({
    first: '',
    last: '',
    email: '',
    confirmEmail: '',
  });

  const [submitting, setSubmitting] = useState(false);
  const [showThankYou, setShowThankYou] = useState(false); // <-- new state

  const handleChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>
  ) => {
    const { name, value } = e.target;
    setForm(f => ({ ...f, [name]: value }));
  };

  const submitCompensation = async (payload: any) => {
    try {
      const response = await fetch('http://localhost:8000/compensation', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });
      if (!response.ok) throw new Error('Submission failed');
      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Submission error:', error);
      return null;
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setSubmitting(true);

    const submissionTime = new Date().toISOString();

    const payload = {
      submissionTime,
      ...form,
    };

    const submitResult = await submitCompensation(payload);
    localStorage.setItem('compensationForm', JSON.stringify(form));
    
    if (!submitResult || submitResult.status !== 'success') {
      alert('Submission failed, please try again.');
      setSubmitting(false);
    } else {
      setShowThankYou(true); // <-- show thank you page
      setSubmitting(false);

      // Save all localStorage data to a txt file on the server before clearing
      try {
        const pid = localStorage.getItem('participantId') || 'unknown_pid';
        const allLocalStorage: Record<string, string> = {};
        for (let i = 0; i < localStorage.length; i++) {
          const key = localStorage.key(i);
          if (key) allLocalStorage[key] = localStorage.getItem(key) || '';
        }
        await fetch('http://localhost:8000/save-data', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            filename: `${pid}_localstorage_backup_${new Date().toISOString().replace(/[:.]/g, '-')}.txt`,
            content: JSON.stringify(allLocalStorage, null, 2)
          })
        });

      } catch (e) {
        // Optionally log error, but do not block clearing
        console.error('Failed to save localStorage backup:', e);
      }

      localStorage.clear();
    }
  };

  // Conditionally render: if showThankYou, show thank you message
  if (showThankYou) {
    return (
      <div className="max-w-2xl mx-auto p-6 flex flex-col items-center justify-center min-h-[60vh]">
        <h2 className="text-2xl font-bold mb-3">Thank you!</h2>
        <p className="text-lg">Your response has been submitted.</p>
      </div>
    );
  }

  return (
    <div className="max-w-4xl mx-auto p-6">
      <h2 className="text-2xl font-bold mb-6">Compensation Questionnaire</h2>
      <div>
        <h3 className="text-gray-600 mb-8 whitespace-pre-line text-left">
          Thank you for completing the task! Please fill out the information below so we can process your compensation.
          Compensation will be provided as an Amazon gift card at a rate of $15 per hour.
        </h3>
      </div>
      <form onSubmit={handleSubmit} className="space-y-12 text-left">
        {/* First */}
        <div>
          <label className="block text-lg font-medium mb-1">First <span className="text-red-500">*</span></label>
          <input
            type="text"
            name="first"
            required
            value={form.first}
            onChange={handleChange}
            className="w-full border rounded p-2"
          />
        </div>
        {/* Last */}
        <div>
          <label className="block text-lg font-medium mb-1">Last <span className="text-red-500">*</span></label>
          <input
            type="text"
            name="last"
            required
            value={form.last}
            onChange={handleChange}
            className="w-full border rounded p-2"
          />
        </div>
        {/* Email */}
        <div>
          <label className="block text-lg font-medium mb-1">Email <span className="text-red-500">*</span></label>
          <input
            type="email"
            name="email"
            required
            value={form.email}
            onChange={handleChange}
            className="w-full border rounded p-2"
          />
        </div>
        {/* Confirm Email */}
        <div>
          <label className="block text-lg font-medium mb-1">Confirm Email <span className="text-red-500">*</span></label>
          <input
            type="email"
            name="confirmEmail"
            required
            value={form.confirmEmail}
            onChange={handleChange}
            className="w-full border rounded p-2"
          />
        </div>
        <button
          type="submit"
          disabled={submitting}
          className="w-full bg-blue-600 text-white rounded py-2 hover:bg-blue-700 disabled:opacity-50"
        >
          {submitting ? 'Submitting…' : 'Done'}
        </button>
      </form>
    </div>
  );
};

export default Compensation;