using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading.Tasks;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Popups;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;

// Die Vorlage "Leere Seite" ist unter http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409 dokumentiert.

namespace RefinerTest
{
    /// <summary>
    /// Eine leere Seite, die eigenständig verwendet oder zu der innerhalb eines Rahmens navigiert werden kann.
    /// </summary>
    /// 

    public sealed partial class MainPage : Page
    {
        public MainPage()
        {
            this.InitializeComponent();
            refiner = new NativeRefinerComponent.NativeRefiner();
        }


        //use async for asyncrone operation
        private async void Test_Button_Click(object sender, RoutedEventArgs e)
        {
            string path = Directory.GetCurrentDirectory();
            var dialog = new MessageDialog("Working path: "+path);
            await dialog.ShowAsync();
            //resultBox.Text = path;
        }


        NativeRefinerComponent.NativeRefiner refiner;

        private async void Open_Button_Click(object sender, RoutedEventArgs e)
        {

     
            var picker = new Windows.Storage.Pickers.FileOpenPicker();
            picker.ViewMode = Windows.Storage.Pickers.PickerViewMode.Thumbnail;
            picker.SuggestedStartLocation =
                Windows.Storage.Pickers.PickerLocationId.ComputerFolder;
            picker.FileTypeFilter.Add(".obj");
            Windows.Storage.StorageFile file = await picker.PickSingleFileAsync();
            if (file != null)
            {
                // Application now has read/write access to the picked file
                //this.resultBox.Text = "Picked object: " + file.Name+" path: " +file.Path;

                refiner.addInitModel(file.Name);
                var dialog = new MessageDialog("Test, added model");
                resultBox.Text= refiner.getSize().ToString()+" triangles";
            }
            else
            {
                this.resultBox.Text = "Operation cancelled.";
            }
        }

        private async void Compute_Click(object sender, RoutedEventArgs e)
        {
            String X = await refiner.Refine();
             var dialog = new MessageDialog("Solution path: " + X);
                await dialog.ShowAsync();
        }

        private async void Open_Image_Button_Click(object sender, RoutedEventArgs e)
        {


            var picker = new Windows.Storage.Pickers.FileOpenPicker();
            picker.ViewMode = Windows.Storage.Pickers.PickerViewMode.Thumbnail;
            picker.SuggestedStartLocation =
                Windows.Storage.Pickers.PickerLocationId.ComputerFolder;
            picker.FileTypeFilter.Add(".png");
            Windows.Storage.StorageFile file = await picker.PickSingleFileAsync();
            string matText = "";
            if (file != null)
            {
                // Application now has read/write access to the picked file
                try
                {
                    matText = System.IO.File.ReadAllText(file.Name + ".matr");
                }
                catch (Exception exc){}
                char[] delimiterChars = { ' ', ',', ':', '\t', '\n' };
                string[] values = matText.Split(delimiterChars, StringSplitOptions.RemoveEmptyEntries);
                float[] fV = new float[32];
                if (values.Length != 32)
                {
                    this.resultBox.Text = "no valid matrices (not right amount of values or no file) length of values is: " + values.Length.ToString();
                    
                }
                else
                {
                    try
                    {
                        int i = 0;
                        foreach (String s in values) {
                            fV[i] = float.Parse(s);
                            i++;
                        }

                    }
                    catch (Exception exc)
                    {
                        this.resultBox.Text = "no valid matrices (not numbers)/" + exc.ToString();
                    }
                }

                System.Numerics.Matrix4x4 View = new System.Numerics.Matrix4x4(fV[0], fV[1], fV[2], fV[3],
                    fV[4], fV[5], fV[6], fV[7],
                    fV[8], fV[9], fV[10], fV[11],
                    fV[12], fV[13], fV[14], fV[15]);

                System.Numerics.Matrix4x4 Projection = new System.Numerics.Matrix4x4(fV[16], fV[17], fV[18], fV[19],
                    fV[20], fV[21], fV[22], fV[23],
                    fV[24], fV[25], fV[26], fV[27],
                    fV[28], fV[29], fV[30], fV[31]);
               // this.resultBox.Text = "View matrix is: " + View.ToString();
                //this.resultBox.Text = "Picked Image: " + file.Name + " path: " + file.Path;
                refiner.addPicture(file.Name, View, Projection);
                this.imageCountBox.Text = refiner.getNImages().ToString() + " images\n";// + View.ToString() + "\n" + Projection.ToString();
            }
            else
            {
                this.resultBox.Text = "Operation cancelled.";
            }
        }
    }
}
