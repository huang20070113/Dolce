// Independent eight-neighbour A* acceptance check; coordinates are PGM pixels.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

struct Image {int w,h;std::vector<unsigned char> data;};
Image read(const char *path) {
  std::ifstream in(path,std::ios::binary);
  std::string magic;int w,h,max;
  if(!(in>>magic>>w>>h>>max)||magic!="P5"||w<=0||h<=0||max!=255)
    throw std::runtime_error("invalid generated PGM");
  in.get();Image im{w,h,std::vector<unsigned char>(static_cast<std::size_t>(w)*h)};
  if(!in.read(reinterpret_cast<char*>(im.data.data()),im.data.size())) throw std::runtime_error("truncated PGM");
  return im;
}
int main(int argc,char **argv) {
  try {
    if(argc!=9) throw std::runtime_error("verify_map final.pgm raw.pgm sx sy gx gy resolution path.csv");
    auto im=read(argv[1]),raw=read(argv[2]);
    if(im.w!=raw.w||im.h!=raw.h) throw std::runtime_error("map dimensions differ");
    int sx=std::stoi(argv[3]),sy=std::stoi(argv[4]),gx=std::stoi(argv[5]),gy=std::stoi(argv[6]);
    double resolution=std::stod(argv[7]);
    auto free=[&](int x,int y) {return x>=0&&y>=0&&x<im.w&&y<im.h&&im.data[y*im.w+x]>250;};
    if(!free(sx,sy)||!free(gx,gy)) throw std::runtime_error("endpoint blocked or outside map");
    int start=sy*im.w+sx,goal=gy*im.w+gx;
    auto heuristic=[&](int n) {int dx=std::abs(n%im.w-gx),dy=std::abs(n/im.w-gy);return 10*std::max(dx,dy)+4*std::min(dx,dy);};
    using Entry=std::pair<int,int>;
    std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
    std::vector<int> g(im.data.size(),std::numeric_limits<int>::max()),parent(im.data.size(),-1);
    g[start]=0;queue.push({heuristic(start),start});
    while(!queue.empty()) {
      auto [f,n]=queue.top();queue.pop();
      if(f!=g[n]+heuristic(n)) continue;
      if(n==goal) break;
      int x=n%im.w,y=n/im.w;
      for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
        if((!dx&&!dy)||!free(x+dx,y+dy)) continue;
        if(dx&&dy&&(!free(x+dx,y)||!free(x,y+dy))) continue;
        int next=(y+dy)*im.w+x+dx,score=g[n]+(dx&&dy?14:10);
        if(score<g[next]) {g[next]=score;parent[next]=n;queue.push({score+heuristic(next),next});}
      }
    }
    if(g[goal]==std::numeric_limits<int>::max()) throw std::runtime_error("NO_PATH");
    std::vector<int> path;
    for(int n=goal;n!=-1;n=parent[n]) path.push_back(n);
    std::reverse(path.begin(),path.end());
    double length=0,clearance=std::numeric_limits<double>::infinity();
    std::ofstream out(argv[8]);out<<"column,row\n";
    for(std::size_t k=0;k<path.size();++k) {
      int x=path[k]%im.w,y=path[k]/im.w;
      out<<x<<','<<y<<'\n';
      if(k) length+=resolution*std::hypot(x-path[k-1]%im.w,y-path[k-1]/im.w);
      // Search 1 metre around the path against the zero-inflation map.
      int radius=static_cast<int>(std::ceil(1.0/resolution));
      for(int dy=-radius;dy<=radius;++dy) for(int dx=-radius;dx<=radius;++dx) {
        int nx=x+dx,ny=y+dy;
        if(nx<0||ny<0||nx>=im.w||ny>=im.h||raw.data[ny*im.w+nx]<1)
          clearance=std::min(clearance,resolution*std::hypot(dx,dy));
      }
    }
    std::cout<<"PATH_FOUND\nCELLS "<<path.size()<<"\nCOST_10_14 "<<g[goal]
             <<"\nLENGTH_M "<<length<<"\nMIN_OBSTACLE_CENTER_DISTANCE_M "<<clearance<<'\n';
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 2;}
}
