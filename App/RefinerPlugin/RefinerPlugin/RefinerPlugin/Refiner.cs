using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;
using UnityEngine;

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

        public void addPicture(String name, UnityEngine.Matrix4x4 viewTransform, UnityEngine.Matrix4x4 projection)
        {
            System.Numerics.Matrix4x4 viewTransformS = new System.Numerics.Matrix4x4(viewTransform.m00, viewTransform.m01, viewTransform.m02, viewTransform.m03,
                viewTransform.m10, viewTransform.m11, viewTransform.m12, viewTransform.m13,
                viewTransform.m20, viewTransform.m21, viewTransform.m22, viewTransform.m23,
                viewTransform.m30, viewTransform.m31, viewTransform.m32, viewTransform.m33);
            System.Numerics.Matrix4x4 projectionS = new System.Numerics.Matrix4x4(projection.m00, projection.m01, projection.m02, projection.m03,
                projection.m10, projection.m11, projection.m12, projection.m13,
                projection.m20, projection.m21, projection.m22, projection.m23,
                projection.m30, projection.m31, projection.m32, projection.m33);
            nativeRefiner.addPicture(name, viewTransformS, projectionS);
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
