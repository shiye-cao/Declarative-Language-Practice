# AI Chain-of-Thought Interface

A modern web application built with React, TypeScript, and Vite for the user study interface for interacting with AI models using chain-of-thought reasoning.

To conduct user study, this web app was deployed using Vercel (https://ai-reasoning-interface.vercel.app/).

---

## Interface Flow & Functionality

### 1. Pre-Study Questionnaire

- Demographics (age, gender, education)
- AI familiarity and usage
- Experience with grad school admissions
- Informed consent collection

### 2. Practice Task

- Non-recorded walkthrough
- Introduces profile layout and decision steps

### 3. Main Task Interface

- 8 randomized student profiles  
- Each profile includes:  
  - **Testing** (GRE scores)  
  - **Academic** (GPA, Major, School Rank)  
  - **Application Materials** (SOP, Diversity Statement, Recommendations)  
- Two-step decision process:
  1. Initial decision (no AI help)
  2. Final decision (with AI chain-of-thought)
- Timing and editing behavior recorded

### 4. Post-Study Questionnaire

- Perceived usefulness & trust in AI
- Understanding of AI reasoning
- Open-ended feedback
- Randomized question order

---

## Data Collection & Privacy

### Local Storage

- Temporarily stores:
  - Decisions & response times
  - Session state
  - Questionnaire responses

### Google Sheets Integration

- Secure logging via Google Apps Script
- JSON payload includes:
  - Condition & task order
  - Full decision logs
  - Timing & editing metadata
  - Questionnaire results

### Data Privacy

- Minimal identifying info collected (optional email)
- Secure and controlled storage in Google Sheets

---

## Tech Stack

- **Frontend Framework**: React 19 with TypeScript
- **Build Tool**: Vite
- **Styling**: Tailwind CSS
- **Routing**: React Router DOM
- **Development Tools**: ESLint, TypeScript
- **Database**: Google Spreadsheet

### Prerequisites

- Node.js (Latest LTS version recommended)
- npm or yarn package manager

### Installation

1. Clone the repository:
```bash
git clone [repository-url]
cd ai-cot-interface
```

2. Install dependencies:
```bash
npm install
```

### Development

To start the development server:
```bash
npm run dev
```

The application will be available at `http://localhost:5173` by default.

### Building for Production

To create a production build:
```bash
npm run build
```

To preview the production build:
```bash
npm run preview
```

## Project Structure

- `src/` - Source code directory
- `public/` - Static assets
- `dist/` - Production build output
- `vite.config.ts` - Vite configuration
- `tsconfig.json` - TypeScript configuration
- `tailwind.config.js` - Tailwind CSS configuration
- `eslint.config.js` - ESLint configuration
- `tsconfig.json` - Base TypeScript configuration
- `tsconfig.app.json` - Application-specific TypeScript configuration
- `tsconfig.node.json` - Node.js specific TypeScript configuration