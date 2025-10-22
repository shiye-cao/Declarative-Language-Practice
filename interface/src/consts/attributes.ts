// src/consts/attributes.ts
export interface AttributeMeta {
  name: string
  description: string
  details: string
}

export const attributeMeta: AttributeMeta[] = [
  {
    name: "GRE Verbal",
    description: "(A value ranging from 130–170)",
    details: "GRE verbal section score. \nStatistics - max: 168, min: 131, mean: 152.45, median: 155.",
  },
  {
    name: "GRE Quant",
    description: "(A value ranging from 130–170)",
    details: "GRE quantitative section score. \nStatistics - max: 170, min: 135, mean: 164.58, median: 167.",
  },
  {
    name: "GRE Writing",
    description: "(A value ranging from 0–6)",
    details: "GRE writing section score. \nStatistics - max: 5, min: 1, mean: 3.7, median: 4.",
  },
  {
    name: "GPA",
    description: "(A value ranging from 0–4.0)",
    details: "Grade point average of the most recent degree. \nStatistics - max: 4, min: 3.02, mean: 3.6151, median: 3.62.",
  },
  {
    name: "Undergrad School Rank",
    description: "(1: top rank, 2: medium rank, 3: low rank)",
    details: "Applicant's most recent institution ranking category. \nStatistics - max: 3, min: 1, mean: 1.53, median: 2.",
  },
  {
    name: "Major",
    description: "Applicant's most recent degree.",
    details: "Applicant's most recent degree received from an institution.",
  },
  {
    name: "SOP Strength",
    description: "(1: weak statement – 5: strong statement)",
    details: "Score of applicant's statement of purpose rated by the admission committee. \nStatistics - max: 5, min: 1, mean: 3.7, median: 4.",
  },
  {
    name: "Diversity Statement",
    description: "(1: weak statement – 5: strong statement)",
    details: "Score of applicant's diveristy statement rated by the admission committee. \nStatistics - max: 4.5, min: 2, mean: 3.44, median: 3.5.",
  },
  {
    name: "Average Recommendation Letter Strength",
    description: "(0: weak letters – 2: strong letters)",
    details: "Average strength of recommendation letters rated by the admission committee. \nStatistics - max: 1.667, min: 0, mean: 1.01667, median: 1.",
  },
]

// just the *values* for each student
export interface StudentProfile {
  id: number
  values: (string|number)[]
}

export const students: StudentProfile[] = [
  { id: 0, values: [ "139", "148", "5", "3.77", "1", "Engineering", "4.5", "4.5", "0.33" ] },
  { id: 1, values: [ "155", "169", "3.5", "3.64", "1", "Business", "3.0", "2.5", "1.67" ] },
  { id: 2, values: [ "152", "168", "1.5", "4.00", "2", "Humanities", "4.5", "3.0", "0.00" ] },
  { id: 3, values: [ "154", "170", "5.0", "3.02", "1", "Natural Science", "1.5", "2.5", "1.67" ] },
  { id: 4, values: [ "159", "168", "2.5", "4.00", "2", "Social Science", "3.0", "3.5", "0.67" ] },
  { id: 5, values: [ "152", "170", "1.5", "3.73", "1", "Humanities", "4.5", "3.5", "1.67" ] },
  { id: 6, values: [ "156", "169", "5.0", "3.74", "1", "Business", "5.0", "3.5", "1.67" ] },
  { id: 7, values: [ "166", "169", "3.5", "3.78", "2", "Humanities", "3.0", "4.0", "1.00" ] },
]

export const practice: StudentProfile[] = [
  { id: 8, values: [ "155", "164", "5", "3.6", "1", "Engineering", "4.5", "2", "0.67" ] }
]

// helper to merge meta + values
export interface Attribute {
  name: string
  description: string
  details: string
  value: string|number
}

export function getAttributesForStudent(id: number): Attribute[] {
  const s = students.find(s=>s.id===id)
  if (!s) return []
  return attributeMeta.map((m, i) => ({
    ...m,
    value: s.values[i] ?? ""
  }))
}

export function getAttributesForPractice(): Attribute[] {
  const s = practice.find(s=>s.id===8)
  if (!s) return []
  return attributeMeta.map((m, i) => ({
    ...m,
    value: s.values[i] ?? ""
  }))
}