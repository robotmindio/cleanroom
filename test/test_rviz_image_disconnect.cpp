#include "../src/safe_image_display.hpp"
#include <QCoreApplication>
#include <stdexcept>

class Display : public lekiwi_rmf::SafeImageDisplay
{
public:
  explicit Display(int& calls) : SafeImageDisplay(nullptr)
  {
    subscription_ = std::make_shared<image_transport::SubscriberFilter>();
    std::weak_ptr<image_transport::SubscriberFilter> target = subscription_;
    subscription_callback_ = message_filters::Connection([target, &calls]() {
      if (target.expired()) throw std::runtime_error("disconnect touched a destroyed image filter");
      ++calls;
    });
  }
  void unsubscribeNow() { unsubscribe(); }
  void disableSubscription() { ITDClass::unsubscribe(); }
};

int main(int argc, char** argv)
{
  QCoreApplication application(argc, argv);
  int calls = 0;
  {
    Display display(calls);
    display.unsubscribeNow();
    display.unsubscribeNow();
  }
  if (calls != 1) throw std::runtime_error("disable/destruction must disconnect exactly once");
  {
    Display display(calls);
  }
  if (calls != 2) throw std::runtime_error("direct destruction must disconnect before deleting the filter");
  {
    Display display(calls);
    // ImageDisplay::onDisable calls its base with a qualified, non-virtual call.
    display.disableSubscription();
    display.unsubscribeNow();
  }
  if (calls != 3) throw std::runtime_error("base disable must not leave a dangling disconnect");
}
