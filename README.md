# MATE ROV 2024
## E-JUST Robotics Club
We are E-JUST Robotics Club. We are passionate about robotics and related fields. Our activities range from conducting workshops to teach students the basics of robotics to participating in competitions and beyond. This README serves as a repository for our work, particularly highlighting our participation in the MATE ROV 2024 competition.

### Table of Contents
1. [Project Overview](#project-overview)
1. [Features](#features)
1. [Design](#design)
1. [Tasks](#tasks)
1. [Folder Structure](#folder-structure)
1. [Where to Upload My Work?](#where-to-upload-my-work)
   


## Project Overview
The MATE ROV competition challenges teams to design and build ROVs to complete various tasks in an underwater environment. We aim to leverage innovative technology and creative solutions to overcome challenges and achieve success in the competition.
## Features
[Adding Them Later]
## Technology Stack
- **Programming Languages** : Python and ETC
- **Frameworks and Libraries** :
  - OpenCV : Function
  - ETC : Function
- **Hardware Components** : ETC
## Design
[Talking about design Later]

## Features
### Graphical User Interface (GUI)
- Description:

### Kinematics System
- Description: 

## Tasks
#### TASK 1: OOI: Coastal Pioneer Array
#### TASK 2: SMART Cables for Ocean Observing 
#### TASK 3: From the Red Sea to Tennessee
#### TASK 4: MATE Floats!


## Demo
[Adding Video Later]
## Team Members
[Adding Them Later]

## Folder Structure
1. `data/`: Stores various datasets and information crucial for the ROV project.
   - `data/ai`: Contains data used for AI algorithms or machine learning models. These datasets may include training, validation, or testing data used to train and evaluate AI models for tasks such as object detection, navigation, or decision-making.
     - `data/ai/external`: Contains datasets and information sourced from external sources, such as publicly available datasets, third-party providers, or other research projects. These datasets may include images, videos, sensor data, or other types of input data used for training and testing AI models.
     - `data/ai/interim`: Serves as an intermediate stage in the data preprocessing pipeline. It typically contains partially processed or temporary data files generated during the data preparation phase.
     - `data/ai/processed`: Stores the finalized and processed datasets ready for training AI models.
   
   - `data/hardware`: Stores data related to hardware components or testing of the ROV.
     - `data/hardware/calibration`: Data used for calibrating sensors or other hardware components to ensure accurate measurements and performance.
     - `data/hardware/sensor-data`: Data collected from various sensors onboard the ROV, such as depth sensors, cameras, temperature sensors, etc.
1. `docs/`: Documentation related to the project, including design documents, technical specifications, and user manuals.

1. `hardware/`: Stores all the hardware-related files, such as electrical schematics, CAD models, and a bill of materials (BoM) detailing the components needed to build the physical ROV.
1. `images/`: Contains the images used in this repo.
1. `models/`: Stores models used for AI training in the ROV project.
   - `models/best-models`: stores the best-performing models selected from the trained models based on certain criteria, such as performance metrics like accuracy, precision, or loss
   - `models/trained-models`: Contains all trained models, including intermediate models generated during the training process.  

1. `notebooks/`: Contains Jupyter notebooks used for various purposes within the ROV project.
1. `results/`: Stores the output, findings, and outcomes generated from various analyses, experiments, or simulations conducted as part of the ROV project. 
1. `src/`: Serves as the main source code folder for the ROV project.
   - `src/3d-model`: Contains the source files and assets related to 3D modeling tasks within the ROV project.
   - `src/autonomous-coral-head`: Contains the source code for the autonomous coral head detection and navigation module of the ROV system.
     - `src/autonomous-coral-head/models`: Contains scripts to train models and then use trained models to make predictions.
     - `src/autonomous-coral-head/visualization`: Contains scripts to create exploratory and results-oriented visualizations.
   - `src/control`: Contains source code related to the control system of the ROV. It contains scripts, modules, or packages responsible for controlling the various aspects of the ROV's behavior, including movement, navigation, sensor interaction, and communication with external devices or systems.
   - `src/gui`: Contains the source code related to the GUI tasks related to the ROV. It contains scripts, modules, or packages responsible for building and managing the GUI components, including windows, buttons, menus, and other interactive elements.


### Where to Upload My Work?
1. 3D Model Team:
   - Upload your scripts to the `src/3d-model` directory.
   - If you have any notebooks, upload them to `src/notebooks/3d-model`
   - Any results, output files, or findings generated from your 3D modeling work should be uploaded to `src/results/3d-model`. This could include rendered images, animations, measurements, or other outcomes of your modeling efforts. Organizing your results in this directory will make it easier for the team to review and utilize your contributions effectively.

1. Autonomous Coral Head Team:
   - Upload your scripts to `src/autonomous-coral-head`. This includes scripts and models for autonomous coral head detection and navigation.
     - All your scripts regarding the YOLO model (training, evaluation, and prediction) should be uploaded to `src/autonomous-coral-head/models`
     - If you have any visualization scripts or tools to visualize the results of your autonomous coral head detection, upload them to  `src/autonomous-coral-head/visualization`
   - Upload your training data set to `data/ai`.
   - Upload your notebooks related to autonomous coral head detection and navigation to `src/notebooks/autonomous-coral-head`.
   - Upload all your trained models to `models/trained-models`. If you are using YOLOv8, it should be the folder `runs/detect/`
   - Upload your best-performing models to `models/best-models`. If you are using YOLOv8, this should be the best `.pt` file you've obtained.
   - Upload your results, output files, or findings from autonomous coral head detection to `srs/results/autonomous-coral-head`. This could include detection results, performance metrics, or any other outcomes of your detection system.

1. Control Team:
   - Upload your work to the `src/control` directory. This includes scripts, modules, or packages related to the control system of the ROV, responsible for controlling movement, navigation, sensor interaction, and communication with external devices or systems.
   - Upload any results, output files, or findings generated from your control-related tasks to `src/results/control`. This could include logs, performance metrics, or any other outcomes of your control system's operation.

1. GUI Team:
   - Upload all scripts, modules, or packages related to building and managing GUI components to `src/gui`. This directory should contain code responsible for creating windows, buttons, menus, and other interactive elements of the ROV's graphical user interface.
     - Upload your front-end scripts and GUI components to `src/gui/frontend`. This directory should contain code responsible for creating visual elements and user interfaces for the ROV's graphical interface.
     - If you're working on any back-end scripts or logic for the GUI, upload them to `src/gui/backend`. This directory should contain code responsible for handling data, communication with servers or databases, and any server-side processing required for the GUI.
   - Add your design files (UI, Styles, Colors, etc) to the `docs/design-docs` directory.




