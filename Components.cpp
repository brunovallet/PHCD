
// Components processing with OpenCv
// Created 31/07/2012 by Bruno Vallet

#include "Components.h"
#include <opencv/highgui.h>
#include <algorithm>
#include <queue>
#include <set>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <time.h>

pixcoord_t operator+(const pixcoord_t & p1, const pixcoord_t & p2)
{
    return pixcoord_t(p1.i+p2.i,p1.j+p2.j);
}
pixcoord_t operator+=(pixcoord_t & p1, const pixcoord_t & p2)
{
    p1.i+=p2.i; p1.j+=p2.j;
    return p1;
}

// implementation of TopoComponent members
TopoComponent::TopoComponent(cv::Mat * _p_id_img, int _id, pixcoordv_t first_pix):
    p_id_img(_p_id_img), id(_id),
    origin(first_pix.val),birth(first_pix.val),death(first_pix.val),
    p_child(NULL),
    eaten(false),is_valid(false),border_need_recompute(true)
{
    v_pixv.push_back(first_pix);
    p_id_img->at<int>(first_pix.i, first_pix.j)=id;
}

void TopoComponent::AddPix(pixcoordv_t pix)
{
    v_pixv.push_back(pix);
    if(pix.val < death) death = pix.val;
    p_id_img->at<int>(pix.i, pix.j)=id;
    border_need_recompute=true;
}

void TopoComponent::AddParent(TopoComponent * p_parent)
{
    vp_parent.push_back(p_parent);
    p_parent->p_child = this;
    if(p_parent->origin > origin) origin = p_parent->origin;
    // this is only necessary for border computation, because border pixels might belong to the parents
    // note that we do not use AddPix because the ids of these pixels in id_img should remain the id of the parent
    for(componentv_t::iterator it = p_parent->v_pixv.begin(); it != p_parent->v_pixv.end(); it++) v_pixv.push_back(*it);
}

int TopoComponent::LastChildId()
{
    if(p_child) return p_child->LastChildId();
    return id;
}

void TopoComponent::Eat(TopoComponent * p_victim)
{
    p_victim->eaten=true;
    for(componentv_t::iterator it=p_victim->v_pixv.begin(); it!=p_victim->v_pixv.end(); it++)
        AddPix(*it);
}

void TopoComponent::ComputeBorder()
{
    if(!border_need_recompute) return;
    v_border.clear();
    for(componentv_t::iterator it=v_pixv.begin(); it!=v_pixv.end(); it++)
    {
        if(it->i == 0 || it->j == 0 || it->i == p_id_img->rows-1 || it->j == p_id_img->cols-1 ||
                p_id_img->at<int>(it->i, it->j-1) > id ||
                p_id_img->at<int>(it->i-1, it->j) > id ||
                p_id_img->at<int>(it->i, it->j+1) > id ||
                p_id_img->at<int>(it->i+1, it->j) > id)
        {
            v_border.push_back(*it);
        }
    }
    border_need_recompute = false;
}

int TopoComponent::Age(bool from_origin)
{
    if(from_origin) return (int)origin-(int)death;
    return (int)birth-(int)death;
}
int TopoComponent::Area()
{
    return v_pixv.size();
}
double TopoComponent::Beauty(double alpha)
{
    ComputeBorder();
    return std::pow(2./v_border.size(), alpha) * std::pow(M_PI, alpha-1) * Area(); // 2. * Area() / v_border.size(); // (FOUR_PI * Area())/(v_border.size()*v_border.size());
}
bool TopoComponent::IsValid(const FindComponentsParam & param)
{
    is_valid = (Age()>param.z_thr && Area()>param.min_comp_size && Beauty()>param.min_beauty);
    if(!is_valid) return false;
    int i_mask=0;
    for(componentv_t::iterator it=v_pixv.begin(); it!=v_pixv.end(); it++)
    {
        if(param.veget_mask->at<ushort>(it->i, it->j) > 0) i_mask++; // positive mask value=inside mask
    }
    is_valid=i_mask < v_pixv.size() * param.max_mask_overlap;
    return is_valid;
}

bool TopoComponent::HasValidChild()
{
    if(p_child)
    {
        if(p_child->is_valid) return true;
        return p_child->HasValidChild();
    }
    return false;
}

void TopoComponent::Print()
{
    std::cout << id << ":" << origin << "-" << birth << "-" << death << (eaten?"eaten":"") << std::endl;
}

// other functions
void FindComponents(cv::Mat & img, v_component_t & big, v_component_t & filtered, const FindComponentsParam & param)
{
    // 0=background, 1=in process, param.in_obj=objects, n>param.in_obj=component
    int i_comp = param.in_obj+1, n_comp=0;
    for( int i = 0; i < img.rows; i++ ) for( int j = 0; j < img.cols; j++ )
    {
        if(img.at<ushort>(i, j) == param.in_obj) // found an unprocessed object
        {
            component_t comp;
            std::queue<pixcoord_t> pix_to_process;
            pix_to_process.push(pixcoord_t(i,j));
            img.at<ushort>(i, j) = 1;
            while(!pix_to_process.empty())
            {
                pixcoord_t top = pix_to_process.front();
                pix_to_process.pop();
                img.at<ushort>(top.i, top.j) = i_comp;
                comp.push_back(top);

                // fill in 4-connexity
                pixcoord_t N(top.i+1, top.j),S(top.i-1, top.j),E(top.i, top.j+1),W(top.i, top.j-1);
                if(top.i>0 && img.at<ushort>(S.i, S.j) == param.in_obj)
                {
                    pix_to_process.push(S);
                    img.at<ushort>(S.i, S.j) = 1;
                }
                if(top.j>0 && img.at<ushort>(W.i, W.j) == param.in_obj)
                {
                    pix_to_process.push(W);
                    img.at<ushort>(W.i, W.j) = 1;
                }
                if(top.i<img.rows-1 && img.at<ushort>(N.i, N.j) == param.in_obj)
                {
                    pix_to_process.push(N);
                    img.at<ushort>(N.i, N.j) = 1;
                }
                if(top.j<img.cols-1 && img.at<ushort>(E.i, E.j) == param.in_obj)
                {
                    pix_to_process.push(E);
                    img.at<ushort>(E.i, E.j) = 1;
                }
            }
            if(comp.size() > param.min_comp_size)
            {
                // find border pixels
                component_t border;
                for(component_t::iterator it=comp.begin(); it!=comp.end(); it++)
                    if(it->i == 0 || it->j == 0 || it->i == img.rows-1 || it->j ==  img.cols-1 ||
                            img.at<ushort>(it->i, it->j-1) != i_comp ||
                            img.at<ushort>(it->i-1, it->j) != i_comp ||
                            img.at<ushort>(it->i, it->j+1) != i_comp ||
                            img.at<ushort>(it->i+1, it->j) != i_comp)
                        border.push_back(*it);
                big.push_back(comp);
                bool beautiful = true;
                if(param.min_beauty>0)
                {
                    double beauty = 4.*M_PI * comp.size() / (border.size()*border.size());
                    bool beautiful = (beauty > param.min_beauty);
                }
                //if(i_comp%10 == 0) std::cout << "Component " << i_comp << "->" << border.size() << "/" << comp.size()
                //    << ": " << (beautiful?"beautiful:":"ugly:") << beauty << std::endl;
                if(beautiful) filtered.push_back(comp); // found a component big and beautiful enough
            }
            i_comp++;
        }
    }
}

void FindComponents(cv::Mat & img, v_component_t & comp)
{
    FindComponentsParam param;
    v_component_t dummy;
    FindComponents(img, comp, dummy, param);
}

cv::Mat KeepComponents(const cv::Mat & img, const v_component_t & v_comp)
{
    cv::Mat ret(img.size(), CV_8U, cv::Scalar(0));
    for(v_component_t::const_iterator it_vcomp=v_comp.begin(); it_vcomp!=v_comp.end(); it_vcomp++)
        for(component_t::const_iterator it_comp=it_vcomp->begin(); it_comp!=it_vcomp->end(); it_comp++)
            ret.at<uchar>(it_comp->i, it_comp->j) = 255; // img.at<ushort>(it_comp->i, it_comp->j);
    return ret;
}

cv::Mat KeepTopoComponents(const cv::Mat & img, const std::vector< std::vector<TopoComponent*> > & vv_comp)
{
    cv::Mat ret(img.size(), img.type(), cv::Scalar(0));
    for(std::vector< std::vector<TopoComponent*> >::const_iterator it_vcomp=vv_comp.begin(); it_vcomp!=vv_comp.end(); it_vcomp++)
        for(std::vector< TopoComponent*>::const_iterator it_comp=it_vcomp->begin(); it_comp!=it_vcomp->end(); it_comp++)
        {
            if((*it_comp)->vp_parent.empty() && !(*it_comp)->eaten)
                for(componentv_t::iterator pix_it=(*it_comp)->v_pixv.begin(); pix_it!=(*it_comp)->v_pixv.end(); pix_it++)
                    ret.at<ushort>(pix_it->i, pix_it->j)=img.at<ushort>(pix_it->i, pix_it->j);
        }
    return ret;
}

void DrawTopoComponents(cv::Mat & img, const std::vector< std::vector<TopoComponent*> > & vv_comp, int ch, bool filtered, bool border)
{
    std::vector<cv::Mat> planes;
    split(img, planes);
    for(std::vector< std::vector<TopoComponent*> >::const_iterator it_vcomp=vv_comp.begin(); it_vcomp!=vv_comp.end(); it_vcomp++)
    {
        for(std::vector< TopoComponent*>::const_iterator it_comp=it_vcomp->begin(); it_comp!=it_vcomp->end(); it_comp++)
        {
            if(!(*it_comp)->eaten && (!filtered || ((*it_comp)->is_valid && !(*it_comp)->HasValidChild())))
            {
                componentv_t pix_to_draw = (border?(*it_comp)->v_border:(*it_comp)->v_pixv);
                for(componentv_t::iterator it=pix_to_draw.begin(); it!=pix_to_draw.end(); it++)
                {
                    planes[ch].at<uchar>(it->i, it->j) = 255;
                }
            }
        }
    }
    cv::merge(planes, img);
}

void DrawFilteredTopoComponentsBorders(cv::Mat & img, const std::vector< std::vector<TopoComponent*> > & vv_comp)
{
    std::vector<cv::Mat> planes;
    split(img, planes);
    for(std::vector< std::vector<TopoComponent*> >::const_iterator it_vcomp=vv_comp.begin(); it_vcomp!=vv_comp.end(); it_vcomp++)
    {
        for(std::vector< TopoComponent*>::const_iterator it_comp=it_vcomp->begin(); it_comp!=it_vcomp->end(); it_comp++)
        {
            if(!(*it_comp)->eaten && (*it_comp)->is_valid && !(*it_comp)->HasValidChild())
            {
                for(componentv_t::iterator it=(*it_comp)->v_border.begin(); it!=(*it_comp)->v_border.end(); it++)
                {
                    planes[1].at<uchar>(it->i, it->j) = 255;
                }
            }
        }
    }
    cv::merge(planes, img);
}

cv::Mat KeepTopoComponents(const cv::Mat & img, const std::vector<TopoComponent*> & v_comp)
{
    cv::Mat ret(img.size(), img.type(), cv::Scalar(0));
    for(std::vector< TopoComponent*>::const_iterator it_comp=v_comp.begin(); it_comp!=v_comp.end(); it_comp++)
    {
        if((*it_comp)->vp_parent.empty() && !(*it_comp)->eaten)
            for(componentv_t::iterator pix_it=(*it_comp)->v_pixv.begin(); pix_it!=(*it_comp)->v_pixv.end(); pix_it++)
                ret.at<ushort>(pix_it->i, pix_it->j)=img.at<ushort>(pix_it->i, pix_it->j);
    }
    return ret;
}

// functor for std::sort
bool TopDownImgValueFunctor(const pixcoordv_t& pix1, const pixcoordv_t& pix2){return (pix1.val>pix2.val);}

#define VISU_STEP 100 // shoot every VISU_STEP pix
#define NO_ID 2147483647 // largest int, corresponds to pixel outside a component
std::vector< std::vector<TopoComponent*> > FindTopoComponents(cv::Mat & img, v_component_t & v_input_component, const FindComponentsParam & param)
{
    cv::Mat id_img(img.size(), CV_32S, cv::Scalar(NO_ID)); // current component ids, int allows for up to NO_ID=2147483647 components
    std::vector< std::vector<TopoComponent*> > ret;
    // main loop on input components
    std::cout << "Processing " << v_input_component.size() << " components";
    for(v_component_t::iterator input_comp_it = v_input_component.begin();
        input_comp_it != v_input_component.end(); input_comp_it++)
    {
        // sort component pixels according to their value in img
        // top_down_pixv_t sorted_pixv; // a posteriori sorting is faster
        componentv_t comp_pixv;
        int imax=0, imin=img.rows, jmax=0, jmin=img.cols;
        for(component_t::iterator pix_it = input_comp_it->begin();
            pix_it != input_comp_it->end(); pix_it++)
        {
            comp_pixv.push_back(pixcoordv_t(*pix_it, img.at<ushort>(pix_it->i, pix_it->j)));
            if(pix_it->i > imax) imax =  pix_it->i;
            if(pix_it->i < imin) imin =  pix_it->i;
            if(pix_it->j > jmax) jmax =  pix_it->j;
            if(pix_it->j < jmin) jmin =  pix_it->j;
        }
        std::sort(comp_pixv.begin(), comp_pixv.end(),
                  TopDownImgValueFunctor);
        std::vector<TopoComponent*> vp_topo_component;

        // loop on increasing pix values
        int i_comp = input_comp_it - v_input_component.begin(), i_pix=0;
        bool interesting = (jmin<873 && imin<1162 && jmax>873 && imax>1162) && i_comp>1;
        for(componentv_t::iterator pix_it = comp_pixv.begin();
            pix_it != comp_pixv.end(); pix_it++, i_pix++)
        {
            std::set<int> all_nids, nids; // ids of neighboring TopoComponents and filtered version
            int nid=-1; // current neighbor id
            bool im_ok = pix_it->i > 0, jm_ok = pix_it->j > 0;
            bool ip_ok = pix_it->i < img.rows-1, jp_ok = pix_it->j < img.cols-1;
            if(im_ok) all_nids.insert(id_img.at<int>(pix_it->i-1, pix_it->j));
            if(jm_ok) all_nids.insert(id_img.at<int>(pix_it->i, pix_it->j-1));
            if(ip_ok) all_nids.insert(id_img.at<int>(pix_it->i+1, pix_it->j));
            if(jp_ok) all_nids.insert(id_img.at<int>(pix_it->i, pix_it->j+1));
            if(false) // 8 connexity
            {
                if(im_ok && jm_ok) all_nids.insert(id_img.at<int>(pix_it->i-1, pix_it->j-1));
                if(ip_ok && jm_ok) all_nids.insert(id_img.at<int>(pix_it->i+1, pix_it->j-1));
                if(im_ok && jp_ok) all_nids.insert(id_img.at<int>(pix_it->i-1, pix_it->j+1));
                if(ip_ok && jp_ok) all_nids.insert(id_img.at<int>(pix_it->i+1, pix_it->j+1));
            }
            for(std::set<int>::iterator nit = all_nids.begin(); nit!=all_nids.end(); nit++)
                if(*nit!=NO_ID) nids.insert(vp_topo_component[*nit]->LastChildId());
            if(nids.size() == 0) // pix has no neighboring component => component birth
            {
                vp_topo_component.push_back(new TopoComponent(&id_img, vp_topo_component.size(), *pix_it));
            }
            else if(nids.size() == 1) // pix has one neighboring component => component growth
            {
                vp_topo_component[*nids.begin()]->AddPix(*pix_it);
            }
            else // pix has more than one neighboring component => component merge
            {
                // if the smallest component is younger than a threshold, merge it to the bigger one
                // find oldest component
                TopoComponent * p_oldest = vp_topo_component[*nids.begin()];
                for(std::set<int>::iterator nit = nids.begin(); nit!=nids.end(); nit++)
                {
                    if(vp_topo_component[*nit]->Age() > p_oldest->Age()) p_oldest=vp_topo_component[*nit];
                }
                int n_eaten=0;
                TopoComponent * p_youngest = vp_topo_component[*nids.begin()];
                do // oldest component eats all components too young
                {
                    // find youngest non eaten component
                    for(std::set<int>::iterator nit = nids.begin(); nit!=nids.end(); nit++)
                    {
                        TopoComponent * p_cur_comp = vp_topo_component[*nit];
                        if(p_cur_comp != p_oldest && !p_cur_comp->eaten && p_cur_comp->Age() <= p_youngest->Age())
                            p_youngest=p_cur_comp;
                    }
                    // oldest eat youngest if not valid
                    if(p_youngest->Age()<param.z_thr) // !p_youngest->IsValid(param)
                    {
                        p_oldest->Eat(p_youngest);
                        n_eaten++;
                    }
                } while(p_youngest->eaten && n_eaten<nids.size()-1);
                if(nids.size()-n_eaten>1) // N>1 component deaths->1 component birth mode
                {
                    // create a new component by merging the connected components
                    vp_topo_component.push_back(new TopoComponent(&id_img, vp_topo_component.size(), *pix_it));
                    for(std::set<int>::iterator nid_it = nids.begin(); nid_it != nids.end(); nid_it++)
                    {
                        vp_topo_component.back()->AddParent(vp_topo_component[*nid_it]);
                    }
                } else // only one component, keep living
                {
                    p_oldest->AddPix(*pix_it);
                }
                if(false) // N component deaths->1 component birth mode
                {
                    // create a new component by merging the connected components
                    vp_topo_component.push_back(new TopoComponent(&id_img, vp_topo_component.size(), *pix_it));
                    for(std::set<int>::iterator nid_it = nids.begin(); nid_it != nids.end(); nid_it++)
                    {
                        vp_topo_component.back()->AddParent(vp_topo_component[*nid_it]);
                    }
                }
                if(false) // (N-1) component deaths, no birth mode
                {
                    ushort max_birth = 0;
                    int oldest_comp_id = 0;
                    for(std::set<int>::iterator nid_it = nids.begin(); nid_it != nids.end(); nid_it++)
                    {
                        if(vp_topo_component[*nid_it]->birth > max_birth)
                        {
                            max_birth = vp_topo_component[*nid_it]->birth;
                            oldest_comp_id = *nid_it;
                        }
                    }
                    vp_topo_component[oldest_comp_id]->AddPix(*pix_it);
                    for(std::set<int>::iterator nid_it = nids.begin(); nid_it != nids.end(); nid_it++)
                    {
                        if(*nid_it != oldest_comp_id)
                        {
                            vp_topo_component[oldest_comp_id]->AddParent(vp_topo_component[*nid_it]);
                        }
                    }
                }
            }
            if(false && interesting && i_pix%VISU_STEP == 1)
            {
                int di=imax-imin, dj=jmax-jmin;
                cv::Mat comp_img; // rand color components
                std::vector<cv::Mat> v_mat(3);
                for(int ch=0; ch<3; ch++) v_mat[ch]=cv::Mat(di, dj, CV_8U);
                for(int i=0; i<imax-imin; i++) for(int j=0; j<jmax-jmin; j++)
                {
                    int id=id_img.at<int>(i+imin,j+jmin);
                    v_mat[0].at<unsigned char>(i,j) = (unsigned char)((99496789*id)%255);
                    v_mat[1].at<unsigned char>(i,j) = (unsigned char)((10245675*id)%255);
                    v_mat[2].at<unsigned char>(i,j) = (unsigned char)((35476987*id)%255);
                }
                cv::merge(v_mat, comp_img);
                std::ostringstream oss; oss << "comp" << i_comp << "-" << std::setfill('0') << std::setw(6) << i_pix << ".png";
                cv::imwrite(param.work_dir+"/"+oss.str(), comp_img);
            }
        }
        if(interesting) // draw final components
        {
            cv::Rect comp_bbx(jmin, imin, jmax-jmin, imax-imin);
            cv::Mat comp_img = id_img(comp_bbx);
            comp_img.convertTo(comp_img, CV_8U);
            std::ostringstream oss; oss << "comp"<<i_comp<<".png";
            cv::imwrite(param.work_dir+"/"+oss.str(), comp_img);
        }
        if(false && interesting) // draw persistence diagram (white on black)
        {
            int diag_size = 256;
            cv::Mat persistance_diagram(diag_size, diag_size, CV_8U, cv::Scalar(0));
            for(int i=0; i<diag_size; i++) persistance_diagram.at<unsigned char>(i,i)=127;
            double zoom = (double)diag_size/64536.;
            for(std::vector<TopoComponent*>::iterator comp_it = vp_topo_component.begin(); comp_it != vp_topo_component.end(); comp_it++)
            {
                persistance_diagram.at<unsigned char>(zoom*(*comp_it)->birth, zoom*(*comp_it)->death)=255;
            }
            std::ostringstream oss2; oss2 << "diag"<<i_comp<<".png"; // << "-" << min_death << "-" << max_birth
            cv::imwrite(param.work_dir+"/"+oss2.str(), persistance_diagram);
        }
        // Final computation of outside border and validity
        for(std::vector< TopoComponent*>::const_iterator it_comp=vp_topo_component.begin(); it_comp!=vp_topo_component.end(); it_comp++)
        {
            if(!(*it_comp)->eaten) // loop over non eaten components
            {
                (*it_comp)->ComputeBorder();
                (*it_comp)->IsValid(param);
            }
        }
        std::cout << " " << vp_topo_component.size() << std::flush;
        ret.push_back(vp_topo_component);

        if(false) // find longest (birth to death) component
        {
            int max_length=0, max_size=0;
            for(std::vector<TopoComponent*>::iterator it = vp_topo_component.begin();
                it != vp_topo_component.end(); it++)
            {
                // int length = (int)(*it)->death - (int)(*it)->birth; // min->max version
                int length = (int)(*it)->birth - (int)(*it)->death; // max->min version
                if(length>max_length) {max_length = length; max_size=(int)(*it)->v_pixv.size();}
            }
            if(vp_topo_component.size()>1000) std::cout << vp_topo_component.size() << " TopoComponents in component " << ret.size()
                                                        << ", longest: " << max_length << " of size " << max_size << std::endl;
        }
    }
    std::cout << std::endl;
    id_img.convertTo(id_img, CV_8U);
    cv::imwrite(param.work_dir+"/persistence.png", id_img);
    return ret;
}

// find TopoComponents in the whole image
std::vector<TopoComponent*> FindTopoComponents(cv::Mat & img, const FindComponentsParam & param)
{
    cv::Mat id_img(img.size(), CV_32S, cv::Scalar(NO_ID)); // current component ids
    std::vector< TopoComponent*> vp_topo_component;

    // sort pixels according to their value in img
    top_down_pixv_t sorted_pixv;
    for(int i=0; i<img.rows; i++) for(int j=0; j<img.cols; j++)
        sorted_pixv.insert(pixcoordv_t(i, j, img.at<ushort>(i, j)));
    //std::sort(sorted_pixv.begin(), sorted_pixv.end(), ImgValueComp);

    // loop on increasing pix values
    for(top_down_pixv_t::iterator pix_it = sorted_pixv.begin();
        pix_it != sorted_pixv.end(); pix_it++)
    {
        std::set<int> all_nids, nids; // ids of neighboring TopoComponents and filtered version
        int nid=-1; // current neighbor id
        bool im_ok = pix_it->i > 0, jm_ok = pix_it->j > 0;
        bool ip_ok = pix_it->i < img.rows-1, jp_ok = pix_it->j < img.cols-1;
        if(im_ok) all_nids.insert(id_img.at<int>(pix_it->i-1, pix_it->j));
        if(jm_ok) all_nids.insert(id_img.at<int>(pix_it->i, pix_it->j-1));
        if(ip_ok) all_nids.insert(id_img.at<int>(pix_it->i+1, pix_it->j));
        if(jp_ok) all_nids.insert(id_img.at<int>(pix_it->i, pix_it->j+1));
        if(true) // 8 connexity
        {
            if(im_ok && jm_ok) all_nids.insert(id_img.at<int>(pix_it->i-1, pix_it->j-1));
            if(ip_ok && jm_ok) all_nids.insert(id_img.at<int>(pix_it->i+1, pix_it->j-1));
            if(im_ok && jp_ok) all_nids.insert(id_img.at<int>(pix_it->i-1, pix_it->j+1));
            if(ip_ok && jp_ok) all_nids.insert(id_img.at<int>(pix_it->i+1, pix_it->j+1));
        }
        for(std::set<int>::iterator nit = all_nids.begin(); nit!=all_nids.end(); nit++)
            if(*nit!=NO_ID) nids.insert(vp_topo_component[*nit]->LastChildId());
        if(nids.size() == 0) // component birth
        {
            vp_topo_component.push_back(new TopoComponent(&id_img, vp_topo_component.size(), *pix_it));
        }
        else if(nids.size() == 1) // component growth
        {
            vp_topo_component[*nids.begin()]->AddPix(*pix_it);
        }
        else // component merge or eat
        {
            //for(std::set<int>::iterator nit = all_nids.begin(); nit!=all_nids.end(); nit++) std::cout << *nit << " ";
            //std::cout << std::endl;
            // if the smallest component is younger than a threshold, merge it to the bigger one
            // find oldest component
            TopoComponent * p_oldest = vp_topo_component[*nids.begin()];
            for(std::set<int>::iterator nit = nids.begin(); nit!=nids.end(); nit++)
            {
                //vp_topo_component[*nit]->Print();
                if(vp_topo_component[*nit]->Age() > p_oldest->Age()) p_oldest=vp_topo_component[*nit];
            }
            //std::cout << "Oldest: "; p_oldest->Print();
            int n_eaten=0;
            TopoComponent * p_youngest = vp_topo_component[*nids.begin()];
            do // oldest component eats all components too young
            {
                // find youngest non eaten component
                for(std::set<int>::iterator nit = nids.begin(); nit!=nids.end(); nit++)
                {
                    if(vp_topo_component[*nit] != p_oldest && !vp_topo_component[*nit]->eaten && vp_topo_component[*nit]->Age() <= p_youngest->Age())
                        p_youngest=vp_topo_component[*nit];
                }
                //std::cout << "Young: "; p_youngest->Print();
                // oldest eat youngest if too young
                if(p_youngest->IsValid(param))
                {
                    n_eaten++;
                    p_oldest->Eat(p_youngest);
                }
            } while(p_youngest->eaten && n_eaten<nids.size()-1);
            if(nids.size()-n_eaten>1)
            {
                // N>1 component deaths->1 component birth mode
                // create a new component by merging the connected components
                vp_topo_component.push_back(new TopoComponent(&id_img, vp_topo_component.size(), *pix_it));
                //std::cout << "Creating "; vp_topo_component.back()->Print();
                for(std::set<int>::iterator nid_it = nids.begin(); nid_it != nids.end(); nid_it++)
                {
                    vp_topo_component.back()->AddParent(vp_topo_component[*nid_it]);
                }
            } else // only one component, keep living
            {
                p_oldest->AddPix(*pix_it);
            }
        }
    }
    return vp_topo_component;
}


BiGraphComponent::BiGraphComponent(component_t _comp, cv::Mat * _p_id_img1, cv::Mat * _p_id_img2):
    comp(_comp), p_id_img1(_p_id_img1), p_id_img2(_p_id_img2), id(-1), meta_comp_id(-1)
{
    if(comp.empty()) return;
    id = p_id_img1->at<ushort>(comp[0].i, comp[0].j);
}



void BiGraphComponent::BuildEdges(std::vector<BiGraphComponent*> v_comp)
{
    edge.clear();

    cc_map_t id_coord_map;
    cc_map_t::iterator ccmit;
    for(component_t::iterator it=comp.begin(); it!=comp.end(); it++)
    {
        if( p_id_img1->at<ushort>(it->i, it->j) != id)
            std::cout << "ERROR: img<->component mismatch" << std::endl;

        int id2 = p_id_img2->at<ushort>(it->i, it->j)-256; // id of corresponding comp in img2
        if(id2>0) // overlap pixel
        {
            ccmit = id_coord_map.find(id2);
            if(ccmit == id_coord_map.end()) {
                pixcoordw_t wpc; wpc.sum=1; wpc.sum_i=it->i; wpc.sum_j=it->j;
                id_coord_map.insert(cc_pair_t(id2, wpc));
            }
            else {
                ++(ccmit->second.sum);
                ccmit->second.sum_i+=it->i;
                ccmit->second.sum_j+=it->j;
            }
        }
    }
    //if(id_coord_map.size()>1) std::cout << id-256 << "<->";
    for(ccmit=id_coord_map.begin(); ccmit!=id_coord_map.end(); ccmit++)
    {
        int new_id = ccmit->first;
        BiGraphComponent * new_comp = v_comp[new_id];
        //if(id_coord_map.size()>1) std::cout << new_id << ",";
        BiGraphEdge * new_edge = new BiGraphEdge(this, new_comp);
        new_edge->overlap = ccmit->second;
        edge.push_back(new_edge);
        new_comp->edge.push_back(new_edge);
    }
    //if(id_coord_map.size()>1)  std::cout << std::endl;
}

pixcoord_t BiGraphComponent::Color(int meta_id, int img_id)
{
    if(meta_comp_id>0) return pixcoord_t(0,0);
    meta_comp_id=meta_id;
    pixcoord_t size(1-img_id, img_id);
    for(std::vector<BiGraphEdge*>::iterator it=edge.begin(); it!=edge.end(); it++)
    {
        BiGraphComponent * opposite = (*it)->Opposite(this);
        if(opposite->meta_comp_id == -1) size+=opposite->Color(meta_id, 1-img_id);
    }
    return size;
}

int BiGraphComponent::NonOverlapPerEdge()
{
    if(edge.empty()) return 0;
    int overlap_size=0;
    for(std::vector<BiGraphEdge*>::iterator it=edge.begin(); it!=edge.end(); it++)
    {
        overlap_size += (*it)->overlap.sum;
    }
    return (comp.size()-overlap_size)/edge.size();
}
