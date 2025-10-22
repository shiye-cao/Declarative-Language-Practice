import './App.css'
import { Routes, Route } from "react-router-dom"
import SetUp from './components/pages/SetUp'
import PreStudyQuestionnaire from './components/pages/PreStudyQuestionnnaire'
import PostStudyQuestionnaire from './components/pages/PostStudyQuestionnaire'
import Compensation from './components/pages/Compensation'
import Condition1 from './components/pages/Condition1'
import Experimental from './components/pages/Experimental'
import TaskIntroduction from './components/pages/TaskIntroduction'
import InitialPromptAssistant from './components/pages/InitialPromptAssistant'
import InterfaceIntroduction from './components/pages/InterfaceIntroduction'

function App() {
  return (
    <>
    <Routes>
      <Route path='/' element={<SetUp />}/>
      <Route path='/prestudy' element={<PreStudyQuestionnaire />}/>
      <Route path='/interface' element={<InterfaceIntroduction />}/>
      <Route path='/intro' element={<TaskIntroduction />}/>
      <Route path='/agent' element={<InitialPromptAssistant />}/>
      <Route path='/condition1' element={<Condition1/>}/>
      <Route path='/experimental' element={<Experimental />}/>
      <Route path='/poststudy' element={<PostStudyQuestionnaire/>}/>
      <Route path='/compensation' element={<Compensation/>}/>
    </Routes>
    </>
  )
}

export default App