# **2D Spin Image Generation from 3D Point Cloud Meshes**
This repository contains Python code for generating **spin images** from **3D point cloud meshes**.

## **Overview**
Spin images provide a robust representation for **3D surface matching** by capturing geometric information around a point in a 2D image-like format. This script processes a given **3D point cloud** and generates **spin images** for analysis and matching.

## **Example**
When a **point cloud** of an object, such as the **basin** shown below, is provided, the script **segments the point cloud into three chunks** and generates the corresponding **spin images** as output.

### **Input Point Cloud**
![Input Point Cloud](images/basin.jpg) 

### **Generated Spin Images**
![Generated Spin Images](images/spin.jpg)

## **Implementation Details**
- The script is **well-commented**, explaining the inner workings of the **functions and algorithms** used.
- The methodology follows the principles outlined in **Andrew Johnson’s research on Spin-Images for 3D surface matching**.

## **Reference**
This implementation is based on the paper:

**Andrew Johnson**  
*"Spin-Images: A Representation for 3-D Surface Matching"*, Carnegie Mellon University, 1997.  

## **Citation**
If you use this work, please cite it as follows:

```bibtex
@phdthesis{Johnson-1997-14453,
  author = {Andrew Johnson},
  title = {Spin-Images: A Representation for 3-D Surface Matching},
  year = {1997},
  month = {August},
  school = {Carnegie Mellon University},
  address = {Pittsburgh, PA},
  number = {CMU-RI-TR-97-47},
  keywords = {shape representation, 3-D surface matching, object recognition, spin-images, surface mesh, surface registration, object modeling, scene clutter.},
}
