#include "pch.h"
#include "Class1.h"

#include <ppltasks.h>
#include <concurrent_vector.h>


using namespace NativeRefinerComponent;
using namespace Platform;
using namespace concurrency;
using namespace Platform::Collections;
using namespace Windows::Foundation::Collections;
using namespace Windows::Foundation;
using namespace Windows::UI::Core;




NativeRefiner::NativeRefiner()
{

}

void NativeRefinerComponent::NativeRefiner::reset()
{
	throw ref new Platform::NotImplementedException();
}

void NativeRefinerComponent::NativeRefiner::addPicture(Platform::String^ path, Windows::Foundation::Numerics::float4x4 CameraViewTransform, Windows::Foundation::Numerics::float4x4 CameraProjectionTransform)
{
	throw ref new Platform::NotImplementedException();
}

void NativeRefinerComponent::NativeRefiner::addInitModel(Platform::String^ path)
{
	//convert from managed string :/
	std::wstring fooW(path->Begin());
	std::string upath(fooW.begin(), fooW.end());
	model.loadFile(upath);
	//model.loadFile("D:\\testModels\\cubeobj.obj");
}

Windows::Foundation::IAsyncOperationWithProgress<Platform::String^, double>^ NativeRefinerComponent::NativeRefiner::Refine()
{
	//maybe we use this later with events:
	//auto window = Windows::UI::Core::CoreWindow::GetForCurrentThread();
	//m_dispatcher = window->Dispatcher;
	
	//async task
	return create_async([this](progress_reporter<double> reporter)-> Platform::String^ {
		//do the refining
		throw ref new Platform::NotImplementedException();

		//return path
		return "path";

		reporter.report(100.0);
	});


}

int NativeRefinerComponent::NativeRefiner::getSize() {
	return model.nTriang;
}