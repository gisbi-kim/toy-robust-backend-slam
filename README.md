# toy-robust-backend-slam
- Goal: Toy DCS for understand the importance of a robust back-end system for robust SLAM.

### Notes
- The many parts of this repository's code are barrowed from https://github.com/mpkuse/toy-pose-graph-optimization-ceres
- I added some useful other datasets from [Luca Carlone's site](https://lucacarlone.mit.edu/datasets/) 
- I refactored codes and directory structures for better understanding 
- (To do) I added useful results discussion with repect to DCS On/Off, a portion of outliers, and datasets. 
- (To do) I added code-block-explanation slides.
- (To do) I extended the original 2D example to 3D (i.e., SE3) examples. 

### How to run 
- move to DCS-ceres directory just run 
```
$ ./main DATASET_NAME_WITHOUGH_DOTG2O NUM_OUTLIER_LOOPS DSC_ON
```
- for example, 
```
$ ./do_build.sh INTEL 50 1 # USING DCS
$ ./do_build.sh INTEL 50 0 # NOT USING DCS
```
