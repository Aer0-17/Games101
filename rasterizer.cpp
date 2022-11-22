// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

/*    std::vector<Vector3f> ver, ver2;
    ver.push_back({_v[1].x()-_v[0].x(),_v[1].y()-_v[0].y(),0}); ver2.push_back({x-_v[0].x(),y-_v[0].y(),0});    
    ver.push_back({_v[2].x()-_v[1].x(),_v[2].y()-_v[1].y(),0}); ver2.push_back({x-_v[1].x(),y-_v[1].y(),0});
    ver.push_back({_v[0].x()-_v[2].x(),_v[0].y()-_v[2].y(),0}); ver2.push_back({x-_v[2].x(),y-_v[2].y(),0});

    for(int i=0;i<3;i++){
        if(ver[i].cross(ver2[i]).z() < 0)
            return false;
    }
    return true;
    */
    
    Vector3f p = {x, y, 0};

    Vector3f p0p1 = _v[1] - _v[0];
    Vector3f p0p = p - _v[0];
    Vector3f p1p2 = _v[2] - _v[1];
    Vector3f p1p = p - _v[1];
    Vector3f p2p0 = _v[0] - _v[2];
    Vector3f p2p = p - _v[2];
    //std::cout << "(p0p1.cross(p0p).z()" << p0p1.cross(p0p).z()<< "p1p2.cross(p1p).z()" << p1p2.cross(p1p).z()<<"p2p0.cross(p2p).z()" << p2p0.cross(p2p).z()<<std::endl;
    //std::cout << "_v[0].x()" << _v[0].x()<< "_v[0].y()" << _v[0].y() << "_v[1].x()" << _v[1].x()<< "_v[1].y()" << _v[1].y() << "_v[2].x()" << _v[2].x()<< "_v[2].y()" << _v[2].y() <<std::endl;
    //点叉乘三条边大于0证明在三角形内
    return ((p0p1.cross(p0p).z() > 0) && (p1p2.cross(p1p).z() > 0) && (p2p0.cross(p2p).z() > 0));
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    //[ABx, ACx, PAx] * [ABy, ACy, PAy] ==> 平行于[u,v,1]向量
    Vector3f U1(v[1].x() - v[0].x(), v[2].x() - v[0].x(), v[0].x() - x);
    Vector3f U2(v[1].y() - v[0].y(), v[2].y() - v[0].y(), v[0].y() - y);

    Vector3f U3 = U1.cross(U2);

    float c1 = 1-(U3.x()/U3.z() + U3.y()/U3.z());
    float c2 = U3.x()/U3.z();
    float c3 = U3.y()/U3.z();

    return {c1,c2,c3};



    //float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    //float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    //float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    //return {c1,c2,c3};
}

//pos_buffer 顶点数据 ind_buffer 索引
void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        //三角形三个点的坐标变成齐次坐标再经过mvp转换
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        //视口变换 [-1,1]^2映射到[0,w][0,h] 平移量是1，缩放因子是w/2 h/2
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        //设置三角形三个顶点的坐标
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    //std::array<Vector3f, 2> v1[2] = {v[0].x, v[0].y, v[1].x, v[1].y};
    //std::array<Vector3f, 2> *bbox;
    //bbox = FindBoundingBox(700, 700, v1);

    //包围盒
    int min_x, min_y, max_x, max_y;
    min_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    min_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));

    max_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    max_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));


    // TODO : Find out the bounding box of current triangle.c
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    //Vector3f p = {0, 0, 0};
    std::vector<Eigen::Vector2f> step
    {
        {0.25, 0.25},
        {0.75, 0.25},
        {0.25, 0.75},
        {0.75, 0.75},
    };
    int count;
    for(int x = min_x; x < max_x; x++)
    {
        for(int y = min_y; y < max_y; y++)
        {
            #if 0
            if(insideTriangle((float)x+0.5, (float)y+0.5, t.v))
            {
                //set_pixel(p, t.color);
                float alpha, beta, gamma;
                auto tup = computeBarycentric2D((float)x + 0.5, (float)y + 0.5, t.v);
                std::tie(alpha, beta, gamma) = tup;
                //用重心坐标求每个点的z值
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                //std::cout << "222" << "get_index(x, y)" << get_index(x, y) <<std::endl;
                //std::cout << "depth_buf[get_index(x, y)]"  << depth_buf[get_index(x, y)] << " z_interpolated" << z_interpolated << std::endl;
                if(depth_buf[get_index(x, y)] > -z_interpolated)
                {
                    //std::cout << "x:" << x << " y " << y << " min_x " << min_x << " min_y " << min_y << " max_x " << max_x << " max_y " << max_y << std::endl;
                    
                    Vector3f point = {(float)x, (float)y, z_interpolated};
                    set_pixel(point, t.getColor());
                    depth_buf[get_index(x, y)] = -z_interpolated;
                }
            }
            #endif
            #if 0
            count = 0;
            //MSAA 一个像素分为2x2
            for(int i = 0; i < 4; i++)
            {
                if(insideTriangle((float)x+step[i][0], (float)y+step[i][1], t.v)) {
                    count++;
                }
            }
            if(count > 0) {
                float alpha, beta, gamma;
                auto tup = computeBarycentric2D((float)x + 0.5, (float)y + 0.5, t.v);
                std::tie(alpha, beta, gamma) = tup;
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                //std::cout << "222" << "get_index(x, y)" << get_index(x, y) <<std::endl;
                //std::cout << "depth_buf[get_index(x, y)]"  << depth_buf[get_index(x, y)] << " z_interpolated" << z_interpolated << std::endl;
                if(depth_buf[get_index(x, y)] > -z_interpolated)
                {
                    //std::cout << "x:" << x << " y " << y << " min_x " << min_x << " min_y " << min_y << " max_x " << max_x << " max_y " << max_y << std::endl;
                    
                    Vector3f point = {(float)x, (float)y, z_interpolated};
                    set_pixel(point, t.getColor()*count/4);
                    depth_buf[get_index(x, y)] = -z_interpolated;
                }
            }
            #endif
            count = 0;
            bool judge = false;
            for(int i = 0; i < 4; i++)
            {
                //去黑边MSAA 对颜色也做采样
                if(insideTriangle((float)x+step[i][0], (float)y+step[i][1], t.v)) {
                    
                    float alpha, beta, gamma;
                    auto tup = computeBarycentric2D((float)x+step[i][0], (float)y+step[i][1], t.v);
                    std::tie(alpha, beta, gamma) = tup;
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    //std::cout << "222" << "get_index(x, y)" << get_index(x, y) <<std::endl;
                    //std::cout << "depth_buf[get_index(x, y)]"  << depth_buf[get_index(x, y)] << " z_interpolated" << z_interpolated << std::endl;
                    if(super_depth_buf[get_super_index(x * 2 + i % 2, y * 2 + i / 2)] > z_interpolated) {
                        //std::cout << "x:" << x << " y " << y << " min_x " << min_x << " min_y " << min_y << " max_x " << max_x << " max_y " << max_y << std::endl;
                        judge = true;
                        //Vector3f point = {(float)x, (float)y, z_interpolated};
                        //set_pixel(point, t.getColor()*count/4);
                        super_depth_buf[get_super_index(x * 2 + i % 2, y * 2 + i / 2)] = z_interpolated;
                        super_frame_buf[get_super_index(x * 2 + i % 2, y * 2 + i / 2)] = t.getColor();
                    }
                }
            }
            if(judge) {
                Vector3f point = {(float)x, (float)y, 1};
                Vector3f color = (super_frame_buf[get_super_index(x*2, y*2)] + super_frame_buf[get_super_index(x*2 + 1, y*2)] + super_frame_buf[get_super_index(x*2, y*2 + 1)] + super_frame_buf[get_super_index(x*2 + 1, y*2 + 1)]) / 4;
                set_pixel(point, color);
            }
        }

    }

}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(super_frame_buf.begin(), super_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(super_depth_buf.begin(), super_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    super_frame_buf.resize(w * h * 4);
    super_depth_buf.resize(w * h * 4);
}

//深度值放在一维数组中，但像素位置相当于二维坐标
//(y-1) * x + x
int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
    //return (y - 1)*width + x;
}

int rst::rasterizer::get_super_index(int x, int y)
{
    return (height*2-1-y)*width*2 + x;
    //return (y - 1)*width*2 + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on