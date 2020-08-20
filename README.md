#Rectangling Panoramic Images via Warping

[Rectangling Panoramic Images via Warping](http://kaiminghe.com/sig13/)的论文复现 C++实现

因为南开某实验室的考核做的...

(improved) seam carving的实现结果有点问题，有时会产生重复插入引起最终插值结果在Opengl纹理贴图上的扭曲，至今无法解决。  
经排查大致认为扭曲撕裂的原因是这个

原论文所说  
>If we represent the two end points of a line segment as a bilinear interpolation of its quad vertexes Vq, we can write e as a linear function of Vq.  
指的应该是[逆双线性插值](https://www.iquilezles.org/www/articles/ibilinear/ibilinear.htm)吧，我也不知道是不是...  
>In this case E is a quadratic function on V and can be optimized via solving a linear system.  
此处我的实现是转成最小二乘估计的形式用最小二乘求解，不知道原论文是否是这个意思