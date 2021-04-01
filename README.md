# PHCD Persistance Homologique pour la Detection de Changements
Code for the paper:
Bruno Vallet. Homological persistence for shape based change detection between Digital Elevation Models.
ISPRS Annals of Photogrammetry, Remote Sensing and Spatial Information Sciences, 2013, II-3/W2, pp.49-54. ⟨10.5194/isprsannals-II-3-W2-49-2013⟩. ⟨hal-02552489⟩

Requirements: OpenCV

Compile:
mkdir build
cd build
cmake ..
make

Test:
./phdc ../data MNE1.png MNE2.png

Parameters:
work_dir dem1 dem2 [sigma_gauss=4(px)] [dilate_radius=4(px)] [erode_radius=4(px)] [z_thr=40] [min_comp_size=50] [min_beauty=0.6] [max_mask_overlap=0.2] [visu_mult=8]

work_dir: where all ouputs are produced. More important is diff_bin.png, a raster map of positive and negative changes detected.
dem1/2: the two input DEMs, ideally in float. Negative values are interpreted as no_data
sigma_gauss: radius of the gaussian used to smooth the difference map (in px)
dilate_radius, erode_radius: radius of the morphological erosion/dilation (in px)
z_thr: threshold on difference of z value (in whatever unit the input DEMs are)
min_comp_size: minimum size for a change component (in px)
min_beauty: minimum beauty for a change component (in [0=ugly, 1=circle=max beauty])
max_mask_overlap: maximum overlap with vegetation for a change component (a vegetation mask veget.png must be given in the working dir)
visu_mult: a multplicative factor for visualization os the change map
