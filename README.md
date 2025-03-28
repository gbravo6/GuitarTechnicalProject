# GuitarTechnicalProject
# Project Proposal: CMPE2965_A03

### Team Members: 
- Griffin
- Adam
- Damien

---

## Purpose:
This project aims to create an interactive online platform to help beginners learn basic guitar chords, scales, and provide a walkthrough of songs. The system will assist users in understanding guitar techniques through a combination of visual aids and audio feedback, improving learning efficiency.

---

## Proposed Solution:
The system will consist of either a single microcontroller or a microcontroller with multiple slave micros, utilizing the following technologies:
- Force Resistors
- Vibration Sensors

The data logged will include chords, scales, songs, and potentially achievements or statistics for each user. The system will feature a metronome to keep pace with the songs and pitch feedback to help users match the correct notes.

A mobile phone app will display:
- The name of the chords/notes.
- The position of the notes/chords on the music staff.
- A visual representation of where the notes/chords are played on the guitar.
- The full song, highlighting the chords/notes to be played.

---

## Specifications:

### User Interface:
- The user can choose from the following options on the web app: Practice Chords, Learn Scale, Play Song, Upload Song.
- The application will retrieve data for the selected operation and guide the user through the relevant process.

### Practice Chords:
In the "Practice Chords" mode, the app will shuffle through basic/intermediate level chords and display:
- The name of the chord.
- A diagram of where the chord is played on the guitar neck.
- The sound of the chord to familiarize the user with the pitch.
- LED indicators on the guitar neck showing where the user should place their fingers.

Once the user applies pressure to the correct locations on the fretboard, the app will move to the next chord. If the user applies pressure to the incorrect locations, the LED indicators will remain lit and highlight the incorrect positions, guiding the user to the correct ones.

### Communication:
- The system will communicate with the mobile app via Bluetooth Low Energy (BLE) to track user inputs and provide real-time feedback.

---

## Innovation Strategy:

- **Azure**: Used for cloud storage and processing.
- **Entity Framework**: For managing data and user progress.
- **Web App Development (HTML, CSS, JS)**: To create the user interface for chord practice and song playback.
- **BLE**: To communicate between the microcontroller and the mobile app.
- **Microcontroller**: To handle sensor inputs and manage the overall system functionality.

This project pushes the boundaries of traditional guitar learning by integrating technology to offer personalized, interactive experiences. The platform can be expanded with more features, such as achievements or advanced song tutorials.

---

## Knowledge and Skill Gaps:

### Familiar Concepts:
- Basic understanding of web development (HTML, CSS, JS).
- Microcontroller programming.
- BLE communication.

### What Needs to Be Learned:
- Understanding guitar structure and design to provide accurate educational feedback.
- Musical theory to ensure proper chord and note naming.
- New sensor technology.

### Potential Concerns:
- Ensuring accurate and real-time feedback for users.
- Proper integration of sensors with minimal interference with the guitar’s sound.
- Effective troubleshooting of any sensor issues or app bugs.

---

## Resource Requirements:

### Components Needed:
- Microcontroller (Raspberry Pi).
- Pressure/Holographic/Membrane Sensors.
- Android mobile phone for the app.
- Battery power for the system.
- Guitar.
- Guitar tuner (to ensure proper calibration of the guitar).
- Guitar sleeve/alterations to incorporate components without affecting the sound quality.

---

## Resources Used:

### Manuals:
- Raspberry Pi Pico documentation for microcontroller programming.
- Sensor datasheets and guides for integration.

### Websites:
- Tutorials on BLE communication.
- Online resources for web development (MDN Web Docs).

### Programming Languages:
- HTML, CSS, and JavaScript, SQL, C# for web app development.
- C/C++ for microcontroller programming.

---

## Project Timeline:

| **Week**  | **Tasks** |
|-----------|-----------|
| **Week 1** | - Course Milestone: Project Team Formation<br> - Start Research on required components (microcontroller, sensors, etc.)<br> - Finalize Project Proposal and prepare the final draft for submission by the end of Week 2. |
| **Week 2** | - Submit Proposal - Draft (Middle of Week)<br> - Finalize the proposal, detailing the project scope, technical specs, and objectives.<br> - Complete Research on sensors (force resistors, holographic sensors, etc.) and finalize selection.<br> - Start Basic Mobile App Design (UI/UX) and outline core functionalities.<br> - Submit Final Proposal - End of Week. |
| **Week 3** | - Proposal Presentation: Present your project proposal and receive feedback.<br> - Finalize Technical Specifications (Draft) based on the proposal presentation feedback.<br> - Start Developing Mobile App: Begin coding the basic app structure (HTML, CSS, JS).<br> - Draft Verification Test to ensure the app and hardware components work together. |
| **Week 4** | - Course Milestone: Technical Specification Approval Deadline.<br> - Final Technical Specification Submission (End of Week).<br> - Begin Hardware Development: Integrate sensors (force resistors, membrane pads) with the guitar.<br> - Start BLE (Bluetooth Low Energy) Communication Testing between microcontroller and mobile app.<br> - Start Initial System Testing: Ensure data flow from hardware to app. |
| **Weeks 5-8** | - **Hardware & Software Development (Main Development Phase)**:<br>   - Develop Sensor Integration: Test and refine the integration of sensors with the guitar (pressure sensors, holographic sensors).<br>   - Refine Mobile App: Finalize core functionality for chord/scale practice and song walkthrough.<br>   - Real-Time Feedback System: Implement feedback mechanisms (real-time tracking, visual/audio feedback).<br>   - Test and Debug the System: Test app with real users to ensure accurate feedback and usability.<br>   - Continued BLE Communication Testing: Ensure BLE works smoothly for real-time feedback. |
| **Week 9** | - Project Synopsis Submission (Summary of progress and challenges).<br> - Progress Report Submission: Provide detailed updates on hardware/software development progress.<br> - Finalize Integration Testing: Test the complete system, including sensor response, app communication, and user interaction.<br> - UI/UX Testing: Refine app based on user feedback, ensuring it is beginner-friendly. |
| **Week 10** | - **Final Refinements and Debugging**:<br>   - Final bug fixes for the app and sensor integration.<br>   - Implement any additional features (e.g., achievements, metronome).<br>   - Prepare for Integration Testing: Full integration of hardware and app for final system testing. |
| **Week 11** | - Final System Testing: Ensure that all components work seamlessly together.<br> - Begin Preparation for Final Presentation: Outline the structure and content of the presentation. |
| **Week 12** | - Final Technical Report Draft Submission: Provide a detailed report documenting the entire project, technical details, and challenges faced.<br> - User Manual Creation: Document how the user interacts with the app and the setup process for the hardware. |
| **Week 13** | - Course Milestone: Integration Testing Deadline.<br> - Conduct final integration testing and submit documentation on system functionality.<br> - Finalize and Submit Final Technical Report: Revise and submit the final version of the technical report.<br> - Prepare for Final Presentation: Rehearse and refine the presentation. |
| **Week 14** | - Prototype Marking Deadline: Submit the final prototype for evaluation.<br> - Rehearse Final Presentation: Final preparation and presentation rehearsals. |
| **Week 15** | - Final Presentation: Present the completed project to the class, demonstrating the system’s functionality and user experience.<br> - Project Showcase: Display the completed project to instructors, peers, and external stakeholders. |

![Alt text](images/FlowDiagram.drawio.png)
