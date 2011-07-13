/*
 *  untitled.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 26/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_TEST_APP_
#define _DMX_TEST_APP_

#include "App.h"
#include "TestModel.h"
#include "TestView.h"

#include "cinder/app/AppBasic.h"

class TestApp : public dmx::App
{
public:

  TestApp(TestModel* model)
  {
    m_model = model;
    m_view = new TestView(model);
  };
  
};


#endif