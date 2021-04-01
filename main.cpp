
// Change detection with OpenCv
// Created 31/07/2012 by Bruno Vallet

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "CvUtils.h"
#include "Components.h"
#include <set>

cv::Mat CvDisk(float radius)
{
    int i_radius = (int)radius;
    int i_diameter = 2*i_radius+1;
    float squared_radius = radius*radius;
    cv::Mat disk(i_diameter, i_diameter, CV_8U, cv::Scalar(0));
    for(int i=0; i<i_diameter; i++) for(int j=0; j<i_diameter; j++)
    {
        float di=i-i_radius, dj=j-i_radius;
        if(di*di + dj*dj < squared_radius) disk.at<uchar>(i,j)=255;
    }
    return disk;
}

int main( int argc, char** argv )
{
    if(argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " work_dir dem1 dem2 "
        << "[sigma_gauss=4(px)] [dilate_radius=4(px)] [erode_radius=4(px)] "
        << "[z_thr=40] [min_comp_size=50] [min_beauty=0.6] [max_mask_overlap=0.2] [visu_mult=8]"
        << std::endl << "Detects changes between dem1 and dem2" << std::endl;
        return 1;
    }
    int i_param=1;
    FindComponentsParam param;
    param.work_dir=argv[i_param++];
    param.in_obj = 255;
    param.z_thr = 40;
    param.min_comp_size = 50;
    param.min_beauty = 0.5;
    param.max_mask_overlap = 0.2;
    std::string dem1_name(argv[i_param++]), dem2_name(argv[i_param++]);
    float sigma_gauss = 4., dilate_radius=4., erode_radius = 4., visu_mult = 8.;

    if(argc>i_param) sigma_gauss = std::atof(argv[i_param++]);
    if(argc>i_param) dilate_radius = std::atof(argv[i_param++]);
    if(argc>i_param) erode_radius = std::atof(argv[i_param++]);
    if(argc>i_param) param.z_thr = std::atof(argv[i_param++]);
    if(argc>i_param) param.min_comp_size = std::atof(argv[i_param++]);
    if(argc>i_param) param.min_beauty = std::atof(argv[i_param++]);
    if(argc>i_param) param.max_mask_overlap = std::atof(argv[i_param++]);
    if(argc>i_param) visu_mult = std::atof(argv[i_param++]);

    std::string dem1_fullname = param.work_dir+"/"+dem1_name;
    std::cout << "--Reading " << dem1_fullname << "..." << std::endl;
    cv::Mat dem1 = cv::imread(dem1_fullname, -1);
    if(!dem1.data)
    {
        std::cout << "Cannot read " << dem1_name << std::endl;
        return 1;
    }

    std::string dem2_fullname = param.work_dir+"/"+dem2_name;
    std::cout << "--Reading " << dem2_fullname << "..." << std::endl;
    cv::Mat dem2 = cv::imread(dem2_fullname, -1);
    if(!dem2.data)
    {
        std::cout << "Cannot read " << dem2_name << std::endl;
        return 1;
    }
    // sanity checks
    std::cout << "dem1: " << Info(dem1) << ", dem2: " << Info(dem2) << std::endl;
    if(dem1.type() != dem2.type())
    {
        std::cout << "ERROR: input DEMs have different types" << std::endl;
        return 2;
    }
    if(dem1.size() != dem2.size())
    {
        std::cout << "ERROR: input DEMs have different sizes" << std::endl;
        return 3;
    }
    // masks
    cv::Mat undef_mask = (dem1<=0)+(dem2<=0);
    undef_mask.convertTo(undef_mask, CV_16U, 255);

    std::string veget_mask_name = param.work_dir+"/veget.png";
    cv::Mat veget_mask = cv::imread(veget_mask_name, -1);
    bool need_veget=true;
    if(!veget_mask.data)
    {
        std::cout << "!!! No veget mask named " << veget_mask_name << " found !!!" << std::endl;
    } else if(veget_mask.size() != dem1.size())
    {
        std::cout << "!!! Veget mask size mismatch: " << Info(veget_mask) << std::endl;
    } else need_veget=false;
    if(need_veget)
    {
        veget_mask = cv::Mat(dem1.size(), CV_16U, cv::Scalar(0));
    } else veget_mask.convertTo(veget_mask, CV_16U, 255);
    param.veget_mask = &veget_mask;

    // ---------- FILTERING ----------
    // closure to remove small holes
    if(dilate_radius)
    {
        std::cout << "--Dilatation by a disk of radius " << dilate_radius << std::endl;
        cv::Mat dilate_disk = CvDisk(dilate_radius);
        cv::dilate(dem1, dem1, dilate_disk);
        cv::dilate(dem2, dem2, dilate_disk);
    }
    if(erode_radius)
    {
        std::cout << "--Erosion by a disk of radius " << erode_radius << std::endl;
        cv::Mat erode_disk = CvDisk(erode_radius);
        cv::erode(dem1, dem1, erode_disk);
        cv::erode(dem2, dem2, erode_disk);
    }

    std::cout << "--Computing diff between DEMs" << std::endl;
    cv::Mat diff21 = (dem2-dem1), diff12 = (dem1-dem2); //-undef_mask-veget_mask;
    //cv::Mat diff21 = dem1, diff12 = dem2;
    std::cout << "diff21: " << Info(diff21) << ", diff12: " << Info(diff12) << std::endl;

    if(sigma_gauss)
    {
        std::cout << "--Blurring with gaussian of sigma=" << sigma_gauss << std::endl;
        int kernel_size = 2*sigma_gauss+1; cv::Size ksize(kernel_size, kernel_size);
        cv::GaussianBlur(diff21, diff21, ksize, sigma_gauss, sigma_gauss);
        cv::GaussianBlur(diff12, diff12, ksize, sigma_gauss, sigma_gauss);
    }

    // ---------- COMPONENTS EXTRACTION ----------
    std::cout << "--Thresholding values under " << param.z_thr << std::endl;
    cv::Mat comp21 = diff21 > param.z_thr;
    cv::Mat comp12 = diff12 > param.z_thr;

    // visualization of binarized diff map
    std::vector<cv::Mat> v_mat_bin(3);
    cv::Mat diff_bin;
    v_mat_bin[0] = comp21; // B
    v_mat_bin[1] = cv::Mat(comp21.size(), comp21.type(), cv::Scalar(0)); // G
    v_mat_bin[2] = comp12; // R
    cv::merge(v_mat_bin, diff_bin);
    std::cout << "diff_bin: " << Info(diff_bin) << std::endl;
    cv::imwrite(param.work_dir+"/diff_bin.png", diff_bin);

    // thresholding converts to uchar (CV_8U), but we compute everything in ushort (CV_16U)
    comp21.convertTo(comp21, CV_16U);
    comp12.convertTo(comp12, CV_16U);

    // find all components and filter using area and beauty
    // Problem: filtering too dependent on z_thr => need to infer thresholds from topology => use of persistance
    std::cout << "--Finding components bigger than " << param.min_comp_size <<
                 " and more beautiful than " << param.min_beauty <<  std::endl;
    v_component_t big21, big12, filtered21, filtered12;
    FindComponents(comp21, big21, filtered21, param);
    FindComponents(comp12, big12, filtered12, param);
    std::cout << big21.size() << "/" << big12.size() << " positive/negative component(s) found" << std::endl;

    std::cout << "--Using persistance to study level set topology with ";
    param.Print();
    std::cout << std::endl;
    std::vector< std::vector<TopoComponent*> > persistance21 = FindTopoComponents(diff21, big21, param);
    std::vector< std::vector<TopoComponent*> > persistance12 = FindTopoComponents(diff12, big12, param);
    //std::vector<TopoComponent*> persistance1 = FindTopoComponents(dem1);
    //std::vector<TopoComponent*> persistance2 = FindTopoComponents(dem2);

    // ---------- VISUALIZATIONS -----------
    std::cout << "--Saving all outputs with visu_mult=" << visu_mult << std::endl;

    // visualization of (filtered or not) components
    cv::Mat img_all21 = KeepComponents(diff21, big21);
    cv::Mat img_all12 = KeepComponents(diff12, big12);
    cv::Mat img_filtered21 = KeepComponents(diff21, filtered21);
    cv::Mat img_filtered12 = KeepComponents(diff12, filtered12);
    img_all21.convertTo(img_all21, CV_8U);
    img_all12.convertTo(img_all12, CV_8U);
    img_filtered21.convertTo(img_filtered21, CV_8U);
    img_filtered12.convertTo(img_filtered12, CV_8U);

    cv::Mat comp; // color coded outlines of components
    std::vector<cv::Mat> v_mat(3);
    v_mat[0] = img_filtered21 + img_filtered12; // B
    v_mat[1] = img_all21; // G
    v_mat[2] = img_all12; // R
    cv::merge(v_mat, comp);
    std::cout << "comp: " << Info(comp) << std::endl;
    cv::imwrite(param.work_dir+"/comp.png", comp);

    // visualization of topo components
    cv::Mat img_topo21 = KeepTopoComponents(diff21, persistance21);
    cv::Mat img_topo12 = KeepTopoComponents(diff12, persistance12);
    //cv::Mat img_topo1 = KeepTopoComponents(dem1, persistance1);
    //cv::Mat img_topo2 = KeepTopoComponents(dem2, persistance2);

    cv::imwrite(param.work_dir+"/topo21.png", img_topo21);
    cv::imwrite(param.work_dir+"/topo12.png", img_topo12);

    // visualization of diff map
    diff21.convertTo(diff21, CV_8U, visu_mult/256.);
    diff12.convertTo(diff12, CV_8U, visu_mult/256.);
    cv::Mat diff; // diff map (green=2>1, red=1>2)
    v_mat[0] = diff21; // B
    v_mat[1] = cv::Mat(diff21.size(), diff21.type(), cv::Scalar(0)); // G
    v_mat[2] = diff12; // R
    cv::merge(v_mat, diff);
    std::cout << "diff: " << Info(diff) << std::endl;
    cv::imwrite(param.work_dir+"/diff.png", diff);

    cv::Mat all_diff(diff.size(), diff.type()), filt_diff(diff.size(), diff.type()), result(diff.size(), diff.type());
    DrawTopoComponents(all_diff, persistance21, 0, false, true);
    DrawTopoComponents(all_diff, persistance12, 2, false, true);
    DrawTopoComponents(filt_diff, persistance21, 0, true, true);
    DrawTopoComponents(filt_diff, persistance12, 2, true, true);
    DrawTopoComponents(filt_diff, persistance21, 1, true, true);
    DrawTopoComponents(filt_diff, persistance12, 1, true, true);

    DrawTopoComponents(result, persistance21, 1, true, false);
    DrawTopoComponents(result, persistance12, 2, true, false);

    std::cout << "all_diff: " << Info(all_diff) << std::endl;
    cv::imwrite(param.work_dir+"/all_diff.png", all_diff);
    cv::imwrite(param.work_dir+"/filt_diff.png", filt_diff);
    cv::imwrite(param.work_dir+"/result.png", result);

    return 0;
}
