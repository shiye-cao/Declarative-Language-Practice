import './App.css'
import { Routes, Route } from "react-router-dom"
import SetUp from './components/pages/SetUp'
import PreStudyQuestionnaire from './components/pages/PreStudyQuestionnnaire'
import SelectScenario from './components/pages/SelectScenario'
import TaskIntroduction from './components/pages/TaskIntroduction'
import InterfaceIntroduction from './components/pages/InterfaceIntroduction'
import Characteristics from "./components/pages/Characteristics"

function App() {
  return (
    <>
    <Routes>
      <Route path='/' element={<SetUp />}/>
      <Route path='/prestudy' element={<PreStudyQuestionnaire />}/>
      <Route path='/select-scenario' element={<SelectScenario />}/>
      <Route path='/interface' element={<InterfaceIntroduction />}/>
      <Route path='/intro' element={<TaskIntroduction />}/>
      <Route path="/characteristics" element={<Characteristics />} />
    </Routes>
    </>
  )
}

export default App