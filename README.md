# SimpleStereotactic
Simple Stereotactic is an unregistered 3D Slicer Extension to give sub-millimeter accuracy for use with the [COMPASS&trade; Stereotactic Frame](http://www.ciimedical.com/index.php?p=compass) coordinate system.

The coordinates describe the parameters necessary to reproduce the targeting. Parameters are consistent with those for a [Compass Stereotactic Frame](http://www.ciimedical.com/index.php?p=compass).  This project can be modified to accomodate other coordinate systems.



## [Installation](INSTALLATION.md): 
See the [installation file](INSTALLATION.md) for complete installation instructions

## Usage
This extension provides functionality to

* Register images with a fiducial frame to the coordinate space of the head frame
* Register an atlas of a given species to the Anterior Commissure and Posterior Commissure (AC/PC) of the provide subject
* Plan and alter multiple trajectories for the placement of canulas to target points.

[Detailed instructions](Example_Instructions.md) using the Porcine_Sample_Data_with_Atlas.mrb are provided.

## Example File
The example file provided, Porcine_Sample_Data_with_Atlas.mrb, is a medical record bundle (mrb) file with the following files zip-compressed within:

* **Sample_Pig_MPRAGE.nrrd**: An MRI of a pig head with z-bar fiducials
* **Scan_Frame_Points.mrk.json**: A sampling of points on the z-bar fiducial in the MRI.
* **porcine_atlas.nrrd**: A numbered porcine atlas (see references).
* **porcine_atlas_color_table.ctbl**: A 3D Slicer color table translating numbers to colors and labels for brain regions of the atlas.
* **Atlas_AC_PC.mrk.json**: Fiducial points marking the Anterior Commissure (AC) and Posterior Commissure (PC) of the porcine atlas.



## References
This software was used in the following paper:

[A novel re-attachable stereotactic frame for MRI-guided neuronavigation and its validation in a large animal and human cadaver model](https://iopscience.iop.org/article/10.1088/1741-2552/aadb49)

Christine A Edwards, Aaron E Rusheen, Yoonbae O2, Seungleal B Paek, Joshua Jacobs, Kristen H Lee, Kendall D Dennis, Kevin E Bennet, Abbas Z Kouzani, Kendall H Lee

[![Alt text](https://img.youtube.com/vi/GGV5B3V5j9A/0.jpg)](https://www.youtube.com/watch?v=GGV5B3V5j9A)

The source of the pig atlas is:

[A three-dimensional digital segmented and deformable brain atlas of the domestic pig](https://doi.org/10.1016/j.jneumeth.2010.07.041)

Stéphan Saikali, Paul Meurice, Paul Sauleau, Pierre-Antoine Eliat,
Pascale Bellaud, Gwenaelle Randuineau, Marc Vérin, Charles-Henri Malbert