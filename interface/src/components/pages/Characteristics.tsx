import React, { useState } from "react"
import { useNavigate } from "react-router-dom"

const Characteristics: React.FC = () => {
  const navigate = useNavigate()

  const [openSections, setOpenSections] = useState<string[]>([])
  const [selectedCharacteristics, setSelectedCharacteristics] = useState<string[]>([])

  const toggleSection = (section: string) => {
    setOpenSections(prev =>
      prev.includes(section)
        ? prev.filter(s => s !== section)
        : [...prev, section]
    )
  }

  const toggleCharacteristic = (characteristic: string) => {
    setSelectedCharacteristics(prev =>
      prev.includes(characteristic)
        ? prev.filter(c => c !== characteristic)
        : [...prev, characteristic]
    )
  }

  const handleNext = () => {
    navigate("/select-scenario", {
      state: { characteristics: selectedCharacteristics }
    })
  }

  return (
    <div className="min-h-screen p-10 bg-white max-w-4xl mx-auto flex flex-col">

      <section className="mb-10">
        <h1 className="text-3xl font-bold mb-4">
          Select Child Characteristics
        </h1>

        <p className="text-lg text-gray-700">
          Choose characteristics that best describe the child you will be interacting with.
        </p>
      </section>


      {/* Autism Spectrum Disorder */}
      <div className="border rounded-lg mb-6">

        <button
          onClick={() => toggleSection("asd")}
          className="w-full text-left px-5 py-4 font-semibold text-lg bg-gray-100 hover:bg-gray-200 rounded-t-lg"
        >
          Autism Spectrum Disorder
        </button>

        {openSections.includes("asd") && (
          <div className="p-5 space-y-3">

            <label className="flex items-center gap-3">
              <input
                type="checkbox"
                checked={selectedCharacteristics.includes("Echolalia")}
                onChange={() => toggleCharacteristic("Echolalia")}
              />
              Echolalia
            </label>

            <label className="flex items-center gap-3">
              <input
                type="checkbox"
                checked={selectedCharacteristics.includes("Scripted Speech")}
                onChange={() => toggleCharacteristic("Scripted Speech")}
              />
              Scripted Speech
            </label>

            <label className="flex items-center gap-3">
              <input
                type="checkbox"
                checked={selectedCharacteristics.includes("Reversal of Pronouns")}
                onChange={() => toggleCharacteristic("Reversal of Pronouns")}
              />
              Reversal of Pronouns
            </label>

          </div>
        )}

      </div>


      {/* Speech Sound Disorder */}
      <div className="border rounded-lg mb-6">

        <button
          onClick={() => toggleSection("ssd")}
          className="w-full text-left px-5 py-4 font-semibold text-lg bg-gray-100 hover:bg-gray-200 rounded-t-lg"
        >
          Speech Sound Disorder
        </button>

        {openSections.includes("ssd") && (
          <div className="p-5 space-y-3">

            <label className="flex items-center gap-3">
              <input
                type="checkbox"
                checked={selectedCharacteristics.includes("Omission of Sounds")}
                onChange={() => toggleCharacteristic("Omission of Sounds")}
              />
              Omissions of Certain Sounds
            </label>

            <label className="flex items-center gap-3">
              <input
                type="checkbox"
                checked={selectedCharacteristics.includes("Addition of Sounds")}
                onChange={() => toggleCharacteristic("Addition of Sounds")}
              />
              Additions of Certain Sounds
            </label>

          </div>
        )}

      </div>


      {/* General Behavior */}
      <div className="border rounded-lg mb-6">

        <button
          onClick={() => toggleSection("behavior")}
          className="w-full text-left px-5 py-4 font-semibold text-lg bg-gray-100 hover:bg-gray-200 rounded-t-lg"
        >
          General Behavior
        </button>

        {openSections.includes("behavior") && (
          <div className="p-5 space-y-3">

            <label className="flex items-center gap-3">
              <input
                type="checkbox"
                checked={selectedCharacteristics.includes("Stubborn")}
                onChange={() => toggleCharacteristic("Stubborn")}
              />
              Stubborn
            </label>

            <label className="flex items-center gap-3">
              <input
                type="checkbox"
                checked={selectedCharacteristics.includes("Not Listening")}
                onChange={() => toggleCharacteristic("Not Listening")}
              />
              Not Listening
            </label>

          </div>
        )}

      </div>


      {/* Navigation Buttons */}
      <div className="flex justify-center gap-4 mt-auto">

        <button
          onClick={() => navigate(-1)}
          className="px-6 py-3 bg-gray-600 text-white rounded hover:bg-gray-700 transition"
        >
          Back
        </button>

        <button
          onClick={handleNext}
          className="px-6 py-3 bg-green-600 text-white rounded hover:bg-green-700 transition"
        >
          Next
        </button>

      </div>

    </div>
  )
}

export default Characteristics