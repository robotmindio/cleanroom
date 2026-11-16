#pragma once

#include <rviz_default_plugins/displays/image/image_display.hpp>

namespace lekiwi_rmf
{
class SafeImageDisplay : public rviz_default_plugins::displays::ImageDisplay
{
public:
  using ImageDisplay::ImageDisplay;
  ~SafeImageDisplay() override { unsubscribe(); }

protected:
  void unsubscribe() override
  {
    if (subscription_) ITDClass::unsubscribe();
    // Jazzy's Connection retains a callback into the destroyed SubscriberFilter.
    // onDisable and the base destructor must not disconnect that callback twice.
    subscription_callback_ = message_filters::Connection();
  }
};
}  // namespace lekiwi_rmf
