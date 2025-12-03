import './App.css'
import { Routes, Route } from "react-router-dom"
import SetUp from './components/pages/SetUp'
import PreStudyQuestionnaire from './components/pages/PreStudyQuestionnnaire'
import TaskIntroduction from './components/pages/TaskIntroduction'
import InterfaceIntroduction from './components/pages/InterfaceIntroduction'

function App() {
  return (
    <>
    <Routes>
      <Route path='/' element={<SetUp />}/>
      <Route path='/prestudy' element={<PreStudyQuestionnaire />}/>
      <Route path='/interface' element={<InterfaceIntroduction />}/>
      <Route path='/intro' element={<TaskIntroduction />}/>
    </Routes>
    </>
  )
}

export default App