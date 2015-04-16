#ifndef GWEN_PROFILE_WINDOW_H
#define GWEN_PROFILE_WINDOW_H

class MyProfileWindow* setupProfileWindow(struct GwenInternalData* data);
void processProfileData(MyProfileWindow* window, bool idle);
void profileWindowSetVisible(MyProfileWindow* window, bool visible);
void destroyProfileWindow(MyProfileWindow* window);

#endif//GWEN_PROFILE_WINDOW_H


