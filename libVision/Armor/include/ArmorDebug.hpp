#include "ArmorSingle.hpp"
namespace wmj
{

  class ArmorDebug
  {
  public:
    int m_debugType;

    Lights m_debugLights;

    Lights m_debugLights_gray;

    Lights m_debugLishtsRoi;

    Armors m_debugArmorsBefore;

    Armors m_debugArmors;

    cv::Mat m_binary;

    cv::Mat m_binary_gray;

    cv::Mat m_src;

    std::vector<cv::Mat> m_numbers;

    void initSet();

    void showLights();

    void showVerticesDistance();

    void showLightRoi();

    void showArmorsBefore();

    void showArmors();

    void showBinary();

    void showNumberRoi();

    void showNumber();

    void deBug(int m_debugType);
  };
}
