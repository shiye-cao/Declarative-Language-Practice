import React, { useState } from 'react'
import { useNavigate } from 'react-router-dom'

interface QuestionnaireData {
  [key: string]: string
}

interface Question {
  id: string
  text: string
  type: 'final' | 'luna' | 'ace'
}


// Section 1: Final Conversation Design
const finalDesignQuestions: Question[] = [
  {
    id: 'satisfied_final',
    text: 'I am satisfied with the final conversation design.',
    type: 'final',
  },
  {
    id: 'confident_final',
    text: 'I feel confident with the final conversation design.',
    type: 'final',
  },
  {
    id: 'expectations',
    text: 'The final conversation design did not meet my expectations.',
    type: 'final',
  },
  {
    id: 'confident_future',
    text: 'I would feel confident designing conversation for another robot in the future.',
    type: 'final',
  },
  {
    id: 'user_satisfied',
    text: 'I think users would be satisfied with their conversation.',
    type: 'final',
  },
  {
    id: 'not_follow_design',
    text: 'The final conversation did not follow my design.',
    type: 'final',
  },
];

// Section 2: Luna (the robot)
const lunaQuestions: Question[] = [
  {
    id: 'luna_easy',
    text: 'Luna was easy to talk to.',
    type: 'luna',
  },
  {
    id: 'luna_not_understand',
    text: 'Luna could not understand me.',
    type: 'luna',
  },
  {
    id: 'luna_enjoy',
    text: 'I enjoyed talking to Luna.',
    type: 'luna',
  },
  {
    id: 'luna_again',
    text: 'I would talk to Luna again in the future.',
    type: 'luna',
  },
];

// Section 3: ACE (the design tool/interface)
const aceQuestions: Question[] = [
  {
    id: 'ace_freq',
    text: 'I think that I would like to use this design tool frequently if I were a conversational robot designer.',
    type: 'ace',
  },
  {
    id: 'ace_complex',
    text: 'I found the design tool unnecessarily complex.',
    type: 'ace',
  },
  {
    id: 'ace_easy',
    text: 'I thought the design tool was easy to use.',
    type: 'ace',
  },
  {
    id: 'ace_support',
    text: 'I think that I would need the support of a technical person to be able to use this design tool.',
    type: 'ace',
  },
  {
    id: 'ace_integrated',
    text: 'I found the various functions in this design tool were well integrated.',
    type: 'ace',
  },
  {
    id: 'ace_inconsistent',
    text: 'I thought there was too much inconsistency in this design tool.',
    type: 'ace',
  },
  {
    id: 'ace_learnquick',
    text: 'I would imagine that most people would learn to use this design tool very quickly.',
    type: 'ace',
  },
  {
    id: 'ace_cumbersome',
    text: 'I found the design tool very cumbersome to use.',
    type: 'ace',
  },
  {
    id: 'ace_confident',
    text: 'I felt very confident using the design tool.',
    type: 'ace',
  },
  {
    id: 'ace_learnbefore',
    text: 'I needed to learn a lot of things before I could get going with this design tool.',
    type: 'ace',
  },
];

const PostStudyQuestionnaire: React.FC = () => {
  // Record landing time in localStorage
  React.useEffect(() => {
    localStorage.setItem('postStudy_landingTime', new Date().toISOString());
  }, []);

  const [form, setForm] = useState<QuestionnaireData>(() => {
    const initial: QuestionnaireData = {};
    [...finalDesignQuestions, ...lunaQuestions, ...aceQuestions].forEach(q => {
      initial[q.id] = '';
    });
    return initial;
  });

  const [submitting, setSubmitting] = useState(false);
  const navigate = useNavigate();

  // Helper to shuffle an array
  function shuffle<T>(array: T[]): T[] {
    const arr = [...array];
    for (let i = arr.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [arr[i], arr[j]] = [arr[j], arr[i]];
    }
    return arr;
  }

  // Shuffle questions in each section only once per session
  const [sections] = useState<
    {
      title: string;
      instruction: string;
      instructionHighlight: string;
      questions: Question[];
    }[]
  >(() => [
    {
      title: 'Reflection on the Final Conversation Design',
      instruction: 'Please respond to these questions with respect to the ',
      instructionHighlight: 'final version of the conversation you designed ONLY.',
      questions: shuffle(finalDesignQuestions)
    },
    {
      title: 'Reflection on Interaction with Luna (the Robot Itself)',
      instruction: 'Please respond to these questions with respect to ',
      instructionHighlight: 'your experience interacting with Luna ONLY.',
      questions: shuffle(lunaQuestions)
    },
    {
      title: 'Reflection on Using the Design Tool/Interface (ACE)',
      instruction: 'Please respond to these questions with respect to ',
      instructionHighlight: 'your experience using the design interface (ACE) ONLY.',
      questions: shuffle(aceQuestions)
    },
  ]);

  const [page, setPage] = useState(0);
  const [showIntro, setShowIntro] = useState(true);

  const handleChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>
  ) => {
    const { name, value } = e.target;
    setForm(f => ({ ...f, [name]: value }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setSubmitting(true);

    // Get meta info from localStorage or elsewhere
    const participantId = localStorage.getItem('participantId') || '';
    const condition = localStorage.getItem('condition') || '';
    const submissionTime = new Date().toISOString();
    const landingTime = localStorage.getItem('postStudy_landingTime') || '';
    const timeTaken = landingTime ? ((new Date(submissionTime).getTime() - new Date(landingTime).getTime()) / 1000).toString() : "0";

    // Build payload to match backend model exactly
    const payload = {
      participantId,
      condition,
      landingTime,
      submissionTime,
      timeTaken,
      satisfied_final: form.satisfied_final,
      confident_final: form.confident_final,
      expectations: form.expectations,
      confident_future: form.confident_future,
      user_satisfied: form.user_satisfied,
      not_follow_design: form.not_follow_design,
      luna_easy: form.luna_easy,
      luna_not_understand: form.luna_not_understand,
      luna_enjoy: form.luna_enjoy,
      luna_again: form.luna_again,
      ace_freq: form.ace_freq,
      ace_complex: form.ace_complex,
      ace_easy: form.ace_easy,
      ace_support: form.ace_support,
      ace_integrated: form.ace_integrated,
      ace_inconsistent: form.ace_inconsistent,
      ace_learnquick: form.ace_learnquick,
      ace_cumbersome: form.ace_cumbersome,
      ace_confident: form.ace_confident,
      ace_learnbefore: form.ace_learnbefore,
    };

    archiveStudy(participantId);

    try {
      // Save to backend
      await fetch('http://localhost:8000/poststudy', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
      });
      localStorage.setItem('postStudyQuestionnaire', JSON.stringify(payload));
      navigate('/compensation');
    } catch (err) {
      console.error(err);
      alert('Submission failed, please try again.');
    } finally {
      setSubmitting(false);
    }
  };

  async function archiveStudy(pid: string) {
    try {
      const res = await fetch("http://localhost:8000/archive-study", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ pid }),
      });
      const data = await res.json();
      if (!res.ok || data.ok === false) {
        console.warn("Archive failed:", data?.error || res.statusText, data?.errors);
      } else {
        console.log(`Archived to ${data.archive_batch}`, data);
      }
    } catch (e) {
      console.error("Archive request error:", e);
    }
  }

  const renderQuestion = (question: Question) => {
    // if (question.type === 'text') {
    //   return (
    //     <fieldset key={question.id} className="space-y-3 mb-6">
    //       <legend className="text-lg font-medium">{question.text}</legend>
    //       <textarea
    //         name={question.id}
    //         value={form[question.id] || ''}
    //         onChange={handleChange}
    //         className="w-full border rounded p-2 min-h-[100px]"
    //         placeholder="Your comments (optional)"
    //       />
    //     </fieldset>
    //   );
    // }
    // // ...existing code for all likert questions...
    return (
      <fieldset key={question.id} className="space-y-3 mb-6">
        <legend className="text-lg font-medium">
          {question.text} <span className="text-red-500">*</span>
        </legend>

        <div className="flex items-center justify-between px-2 mb-1">
          <span className="text-gray-700">Strongly Disagree</span>
          <span className="text-gray-700">Strongly Agree</span>
        </div>

        <div className="flex justify-between max-w-3xl mx-auto">
          {[1, 2, 3, 4, 5].map(n => (
            <label key={n} className="flex flex-col items-center cursor-pointer">
              <span className="text-sm font-medium mb-1">{n}</span>
              <input
                type="radio"
                name={question.id}
                value={String(n)}
                checked={form[question.id] === String(n)}
                onChange={handleChange}
                required
                className="form-radio h-5 w-5 text-blue-600"
              />
            </label>
          ))}
        </div>
      </fieldset>
    );
  };

  return (
    <div className="max-w-10xl mx-auto p-6">
      <h2 className="text-3xl font-bold mb-10">Post-Study Questionnaire</h2>
      {showIntro ? (
        <div className="flex flex-col items-center justify-center min-h-[300px] max-w-8xl mx-auto">
          <p className="mb-10 text-2xl text-left max-w-5xl mx-auto">There are three sections in this survey, one with respect to the <strong>final conversation design</strong>, one with respect to the <strong>robot Luna</strong>, and one with respect to the <strong>design tool ACE</strong>. Please rate your agreement with each statement based on your experience with each aspect respectively.</p>
          <button
            className="bg-blue-600 text-white rounded py-2 px-8 text-lg hover:bg-blue-700"
            onClick={() => setShowIntro(false)}
          >
            Next
          </button>
        </div>
      ) : (
        <form onSubmit={handleSubmit} className="space-y-15 text-left">
          <div className="p-4 border rounded bg-blue-30 mb-5 max-w-5xl mx-auto">
            <div key={String(page)}>
              <h3 className="mb-2 text-xl font-bold">{sections[page].title}</h3>
              <div className="mb-8 whitespace-pre-line">
                {sections[page].instruction}
                <strong>{sections[page].instructionHighlight}</strong>
              </div>
              {sections[page].questions.map(q => renderQuestion(q))}
            </div>
          </div>

          <div className="flex justify-between mt-8">
            <button
              type="button"
              disabled={page === 0}
              className="bg-gray-300 text-gray-700 rounded py-2 px-6 hover:bg-gray-400 disabled:opacity-50"
              onClick={() => setPage(p => Math.max(0, p - 1))}
            >
              Back
            </button>
            {page < sections.length - 1 ? (
              <button
                type="button"
                className="bg-blue-600 text-white rounded py-2 px-6 hover:bg-blue-700"
                onClick={() => setPage(p => Math.min(sections.length - 1, p + 1))}
              >
                Next
              </button>
            ) : (
              <button
                type="submit"
                disabled={submitting}
                className="bg-blue-600 text-white rounded py-2 px-6 hover:bg-blue-700 disabled:opacity-50"
              >
                {submitting ? 'Submitting…' : 'Submit'}
              </button>
            )}
          </div>
        </form>
      )}
    </div>
  );
};

export default PostStudyQuestionnaire;