<div align="center">
    
    
 <div>
  <h1>Optimizing Projection-based Point Cloud Quality Assessment with Human Preferred Viewpoints Selection</h1>
  
_How to select the best viewpoints for projection-based PCQA methods?_

  <div>
      <a href="https://zzc-1998.github.io/" target="_blank">Zicheng Zhang</a><sup>1</sup><sup>*</sup>,
      <a href="" target="_blank">Yu Fan</a><sup>1</sup>,
      <a href="https://scholar.google.com/citations?hl=zh-CN&user=nDlEBJ8AAAAJ" target="_blank">Wei Sun</a><sup>1</sup>,
      <a href="https://minxiongkuo.github.io/" target="_blank">Xiongkuo Min</a><sup>1</sup>,
      <a href="https://scholar.google.ca/citations?user=Tq2hoMQAAAAJ&hl=en" target="_blank">Xiaohong Liu</a><sup>1</sup>,
      <a href="https://github.com/lcysyzxdxc" target="_blank">Chunyi Li</a><sup>1</sup><sup>*</sup>,
      
  </div>

<div>
       <a href="https://teowu.github.io/" target="_blank">Haoning Wu</a><sup>2</sup><sup>*</sup>, 
      <a href="https://personal.ntu.edu.sg/wslin/Home.html" target="_blank">Weisi Lin</a><sup>2</sup>,
      <a href="" target="_blank">Ning Liu</a><sup>1</sup>,
      <a href="https://ee.sjtu.edu.cn/en/FacultyDetail.aspx?id=24&infoid=153&flag=153" target="_blank">Guangtao Zhai</a><sup>1</sup><sup>#</sup>
      
  </div>
  <div>
  <sup>1</sup>Shanghai Jiaotong University,  <sup>2</sup>Nanyang Technological University
       </div>   
<div>
<sup>#</sup>Corresponding author. 
   </div>
  <div style="width: 100%; text-align: center; margin:auto;">
      <img style="width:100%" src="icme_poster.png">
  </div>
  
<div align="left">
    

## Download VP-PCQA Dataset

The dataset can be downloaded via [OneDrive]().

The VP-PCQA.zip is formatted like:
```
|--dis_ply (All the distorted point clouds in .ply format.)
|--dis_vp_image |--AxeGuy_cn_20.ply |--0.png ~ 41.png
(The projection images of 42 viewpoints defined by the geodesic sphere for each distorted point cloud.)
|--dis_vp_json |--AxeGuy_cn_20.ply |--0.json ~ 41.json + vp_attribute.csv
(The camera parameters (.json) for the 42 viewpoints of each distorted point cloud, and preferred viewpoint index features in 'vp_attribute.csv'.)
|--ref_ply (All the reference point clouds in .ply format.)
|--ref_|--ref_ply
(The projection images of 42 viewpoints defined by the geodesic sphere for each reference point cloud.)
```

The human-selected viewpoints can be accessed [here](https://github.com/zzc-1998/PCQA_VP/blob/main/human_preferred_index.csv).
The CSV file lists viewpoints in order of human preference, with the **most favored viewpoints appearing first**.

## Other Utils

`cal_vp_attribute.py' can be used to calculate the preferred index features.

`vp_render.py' can be used to render the projections according to the geodesic sphere.

## Contact

Please contact any of the first authors of this paper for queries.

- Zicheng Zhang, `zzc1998@sjtu.edu.cn`, @zzc-1998


## Citation

If you find our work interesting, please feel free to cite our paper:

```bibtex
@misc{zhang2024abench,
      title={Optimizing Projection-based Point Cloud Quality Assessment with Human Preferred Viewpoints Selection}, 
      author={Zicheng Zhang and Yu Fan and Wei Sun and Xiongkuo Min and Xiaohong Liu and Chunyi Li and Haoning Wu and Weisi Lin and Ning Liu and Guangtao Zhai},
      year={2024},
      booktitle={IEEE International Conference on Multimedia and Expo (ICME)}
}
```
