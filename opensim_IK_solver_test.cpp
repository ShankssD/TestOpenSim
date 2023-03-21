/* -------------------------------------------------------------------------- *
 *                    OpenSim:  TugOfWar1_CreateModel.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Jeffrey A. Reinbolt, Ayman Habib, Ajay Seth, Jack Middleton,    *
 *            Samuel R. Hamner                                                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/*
 *  Below is an example of an OpenSim application that builds a Luxo lamp with
 *  muscles and simulates hopping.*/

// Author:  Matt DeMers

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"
#include "OpenSim/Simulation/VisualizerUtilities.h"
#include "OpenSim/Tools/InverseKinematicsTool.h"
#include <OpenSim/Common/TimeSeriesTable.h>
// #include <OpenSim/Tools/ModelScaler.h>
#include <OpenSim/Tools/ScaleTool.h>

using namespace OpenSim;
using namespace SimTK;

// TO DO: Find a way to set an ideal camera pose
UnitVec3 camera_offset_direction(1, 1, 1);
double camera_offset_distance = 2; // meters

// Intergrator Settings
//----------------------
double sim_time = 5; // seconds

//______________________________________________________________________________
/**
 * main routine to run the model.
 */
int main(int argc, char *argv[])
{   bool showVisualizer{true};

    std::string model_file = "../test_set/UpperLimb_Model_Pose2Sim_Body25b.osim";
    std::string scaleSet_file = "../test_set/UpperLimb_Scaling_Setup_Pose2Sim_Body25b.xml";
    std::string scaleOutput_file = "../test_set/RESULTS/Model_Pose2Sim_Body25b_scaled.osim";
    std::string ikSetup_file = "../test_set/UpperLimb_IK_Setup_Pose2Sim_Body25b.xml";
    std::string ikOutputMotion_file = "../test_set/RESULTS/ikResult.mot";

    // std::string model_file = "../pose2sim_default/Model_Pose2Sim_Body25b.osim";
    // std::string scaleSet_file = "../pose2sim_default/Scaling_Setup_Pose2Sim_Body25b.xml";
    // std::string scaleOutput_file = "../pose2sim_default/RESULTS/Model_Pose2Sim_Body25b_scaled.osim";
    // std::string ikSetup_file = "../pose2sim_default/IK_Setup_Pose2Sim_Body25b.xml";
    // std::string ikOutputMotion_file = "../pose2sim_default/RESULTS/ikResult.mot";
    try
    {
        // Create an OpenSim model and set its name
        Model mdl = Model(model_file);
        mdl.setName("Body25b");

        std::cout << "Finished Loading skeleton" << std::endl;

        // mdl.setUseVisualizer(showVisualizer);
        mdl.updDisplayHints().set_show_frames(true);

        // Pose the model
        State &state = mdl.initSystem();
        std::cout << "State initialized." << std::endl;

        // Show Static Model

        
        // Scale Model
        ModelScaler scaler;
        scaler.setScaleSetFile(scaleSet_file);
        scaler.setOutputModelFileName(scaleOutput_file);
        scaler.setPrintResultFiles(true);
        scaler.setApply(true);
        scaler.processModel(&mdl);

        // Use ScaleTool
        // scaler.run();
        // Model scaled = *(scaler.createModel());


        std::cout << "IK initialized." << std::endl;
        InverseKinematicsTool ikTest = InverseKinematicsTool(ikSetup_file);
        ikTest.setModel(mdl);
        ikTest.setOutputMotionFileName(ikOutputMotion_file);
        ikTest.run();
        std::cout << "IK finished." << std::endl;

        TimeSeriesTable_<SimTK::Real> test{ikOutputMotion_file};
        OpenSim::VisualizerUtilities::showMotion(mdl, test);



        std::cout << "OpenSim example completed successfully.\n";
        // enter anything in the command prompt to quit
        if (showVisualizer)
        {
            std::cout << "Enter anything to quit." << std::endl;
            std::cin.get();
        }
    }
    catch (const OpenSim::Exception &ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }

    return 0;
}
