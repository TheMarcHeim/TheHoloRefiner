using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace RefinerPlugin
{
    public class Refiner
    {
        NativeRefinerComponent.NativeRefiner nativeRefiner;
        Boolean finished = false;

        public Refiner()
        {
            nativeRefiner = new NativeRefinerComponent.NativeRefiner();
        }

        public void reset()
        {
            nativeRefiner.reset();
        }

        public void addPicture(String name, Matrix4x4 viewTransform, Matrix4x4 projection)
        {
            nativeRefiner.addPicture(name, viewTransform, projection);
        }

        public void addInitModel(String path)
        {
            nativeRefiner.addInitModel(path);
        }

        public async void compute()
        {
            //TODO: do more elaborate progress tracking ;)
            finished = false;
            await nativeRefiner.Refine();
            finished = true;
        }
    }
}
