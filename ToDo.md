# ToDos:
- [x] Move installation from README to another file
- [x] Example
- [x] Rename Directories and files appropriately
- [x] Look at FlywheelConnect for the formatting of the CMakeLists.txt files
- [x] linting of python files
    - [x] snake case
    - [x] doc strings
- [ ] Testing the Extension under load
    - [x] Electrodes switching: need to store the entry point as well.
    - [x] Electrodes switching: ensure data is saved for reseting.
    - [x] Dragging target beyond 50-150
        - [ ] Need to update Readme/Wiki to note the constraints.
    - [ ] Address all ToDos.

	
## Non-Essential:
- [ ] Use low-res example files. less than 5 Mb each... if possible...
    - [ ] Reference pig atlas publication
        https://doi.org/10.1016/j.jneumeth.2010.07.041
    - [ ] Reduce size of pig-scan
- [ ] Reduce to a single transform that is built everytime something changes.
- [ ] Ensure that a minimal set of essential frame files are present
    - [ ] Reconstruct transform sequence in code... and hide all essential transforms for the frame
    - [ ] Target_Y->Target_Z->Collar->Target_X->Arc
    - [x] Decide on the visibility of certain elements
- [ ] Reformat along canula
    - IGT - Volume Reslice Driver--- Toggle this option.
    - This is going to take a while....
    - This would combine efforts from the RT module and the incarnations of FlywheelConnect.
- [ ] Find a fancy frame, if I can
    - Fancy Frame on 4 TB external disk
        /media/josher/518A46475CC1D223/Projects/2017.02.16.StereoTactic.SurgicalPlanning/Data/Testing.04
    - As is the Harvard Human Atlas:
        /media/josher/518A46475CC1D223/Projects/BrainDataSets/2017.08.01.Harvard.Brain
    - We don't have an MRI w/ frame for the human atlas.