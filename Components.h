#pragma once

// Components processing with OpenCv
// Created 31/07/2012 by Bruno Vallet

#include "opencv/cv.h"
#include <vector>
#include <iostream>
#include <set>

// structs/classes

/// Pixel coordinates
struct pixcoord_t{int i,j;pixcoord_t(int _i=0, int _j=0):i(_i),j(_j){}};
pixcoord_t operator+(const pixcoord_t & p1, const pixcoord_t & p2);
pixcoord_t operator+=(pixcoord_t & p1, const pixcoord_t & p2);

/// Pixel coordinates and value
struct pixcoordv_t{
    int i,j;ushort val;
    pixcoordv_t(pixcoord_t ij, ushort _val):i(ij.i),j(ij.j),val(_val){}
    pixcoordv_t(int _i, int _j, ushort _val):i(_i),j(_j),val(_val){}
};

/// Weighted sum of pix coords (for barycenter computation)
struct pixcoordw_t
{
    unsigned int sum, sum_i, sum_j;
};

// typedefs
typedef std::vector< pixcoord_t > component_t;
typedef std::vector< pixcoordv_t > componentv_t;
typedef std::vector< component_t > v_component_t;
typedef std::map< unsigned int, pixcoordw_t > cc_map_t;
typedef std::pair< unsigned int, pixcoordw_t > cc_pair_t;

/// Functors to sort pixels according to their value in img
struct TopDownImgValueComp
{
    bool operator()(const pixcoordv_t& pix1, const pixcoordv_t& pix2) const
    {return (pix1.val>pix2.val);}
};
struct BottomUpImgValueComp
{
    bool operator()(const pixcoordv_t& pix1, const pixcoordv_t& pix2) const
    {return (pix1.val<pix2.val);}
};

/// sorted set of pixels, seems that sorting a posteriori (with vector) is faster
typedef std::multiset< pixcoordv_t, TopDownImgValueComp > top_down_pixv_t;

/// parameters for connected components extraction
class FindComponentsParam
{
public:
    /// working directory
    std::string work_dir;
    /// image value of objects to extract (should be at least 2, in practice 255)
    unsigned short in_obj;
    /// threshold on z diff to keep a component
    int z_thr;
    /// min component size in px
    double min_comp_size;
    /// min beauty (4 pi component area/squared border length)
    double min_beauty;
    /// max ratio of the component that can be overlapped by the mask
    double max_mask_overlap;
    /// vegetation mask
    cv::Mat * veget_mask;

    FindComponentsParam():in_obj(255),z_thr(0), min_comp_size(0),min_beauty(0),max_mask_overlap(0), veget_mask(NULL){}
    void Print(){std::cout << "z_thr=" << z_thr << ", min_comp_size=" << min_comp_size << ", min_beauty=" << min_beauty;}
};

/** Extract connected components of pixels in img, stores the big enough in big, and the big AND beautiful enough in filtered
img should be in ushort so we can tag more than 256 components, with only pixels at 0 (background) and param.in_obj (objects)
At the end, background is still at 0 but each connected component has a different label>param.in_obj */
void FindComponents(cv::Mat & img, v_component_t & big, v_component_t & filtered, const FindComponentsParam & param);
/** Same without the big and beatiful thing */
void FindComponents(cv::Mat & img, v_component_t & comp);

/// Topologic component for persistance computation
class TopoComponent
{
public:
    /// pointer to the identifier image
    cv::Mat * p_id_img;
    /// identifier of the component in the id image
    int id;
    /// smaller components are eaten by larger ones
    bool eaten;
    /// Optimization: Last result of call to IsValid() (to use if we are sure we do not need to recompute)
    bool is_valid;
    /// Optimization: flag to know if we need to recompute the border
    bool border_need_recompute;
    /// components which merge generated this TopoComponent
    std::vector<TopoComponent*> vp_parent;
    /// component for which this is a parent (NULL if inexistant)
    TopoComponent* p_child;
    /** levels of origin (oldest ancestor's birth), birth and death of the component
      WARNING: we go from high to low diff values so death < birth */
    ushort origin, birth, death;
    /// pixels in the component
    componentv_t v_pixv;
    /// pixels in the component border
    componentv_t v_border;

    /// only constructor
    TopoComponent(cv::Mat * _p_id_img, int _id, pixcoordv_t first_pix);

    /// Add a pixel to the component
    void AddPix(pixcoordv_t pix);

    /// Add a parent to the component
    void AddParent(TopoComponent * p_parent);

    /// Id of the last child (currently active child)
    int LastChildId();

    /// Eat a component = get all its pixels (stronger than a merge, no parent/child relationship)
    void Eat(TopoComponent * p_victim);

    /// Compute the current component border
    void ComputeBorder();

    /// Age of the component or of its lineage (from_origin)
    int Age(bool from_origin=true);

    /// Component area (number of pixels, including parent pixels)
    int Area();

    /// Component beauty
    double Beauty(double alpha = 1.5);

    /// Does the component pass all the validity tests ? (computes is_valid member)
    bool IsValid(const FindComponentsParam & param);

    /// Does the component have a valid child ?
    bool HasValidChild();

    /// Simple output to console
    void Print();
};

class BiGraphEdge;

/// Topologic component for persistance computation
class BiGraphComponent
{
public:
    /// only constructor, builds edge
    BiGraphComponent(component_t _comp, cv::Mat * _p_id_img1, cv::Mat * _p_id_img2);

    void BuildEdges(std::vector<BiGraphComponent*> v_comp);
    pixcoord_t Color(int meta_id, int img_id);
    int NonOverlapPerEdge();

    /// pointer to the identifier image
    cv::Mat * p_id_img1, * p_id_img2;

    /// pointers to the
    std::vector<BiGraphEdge*> edge;

    /// pixels in the component
    component_t comp;

    /// id (in img1)
    int id;

    /// id of the meta component (component of the intersection graph)
    int meta_comp_id;
};

class BiGraphEdge
{
public:
    BiGraphComponent * comp1, * comp2;
    int color;
    // overlap weighted barycenter
    pixcoordw_t overlap;
    BiGraphEdge(BiGraphComponent * _comp1, BiGraphComponent * _comp2):
        comp1(_comp1), comp2(_comp2), color(1)
    {}
    BiGraphComponent * Opposite(BiGraphComponent * comp){if(comp==comp1) return comp2; return comp1;}
};

cv::Mat KeepComponents(const cv::Mat & img, const v_component_t & v_comp);
cv::Mat KeepTopoComponents(const cv::Mat & img, const std::vector< std::vector<TopoComponent*> > & vv_comp);
void DrawTopoComponents(cv::Mat & img, const std::vector< std::vector<TopoComponent*> > & vv_comp, int ch=1, bool filtered=false, bool border=false);
cv::Mat KeepTopoComponents(const cv::Mat & img, const std::vector<TopoComponent*> & v_comp);

std::vector< std::vector<TopoComponent*> > FindTopoComponents(cv::Mat & img, v_component_t & v_input_component, const FindComponentsParam & param);
std::vector<TopoComponent*> FindTopoComponents(cv::Mat & img);

// Get minima from a mono channel image, unused for now
template<class T> cv::Mat GetMinima(const cv::Mat & img)
{
    cv::Mat minima(img.size(), CV_32S, cv::Scalar(0));
    int mark = 1;
    for( int i = 1; i < img.rows-1; i++ ) for( int j = 1; j < img.cols-1; j++ )
    {
        // 4-connexity
        if(img.at<T>(i, j)<img.at<T>(i+1, j) &&
           img.at<T>(i, j)<img.at<T>(i, j+1) &&
           img.at<T>(i, j)<img.at<T>(i-1, j) &&
           img.at<T>(i, j)<img.at<T>(i, j-1)) minima.at<int>(i,j)=mark++;
    }
    std::cout << mark << " extrema found" << std::endl;
    return minima;
}
