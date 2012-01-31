/*=========================================================================
 *
 *  Copyright David Doria 2011 daviddoria@gmail.com
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

#include "ui_VFHComparisonWidget.h"
#include "VFHComparisonWidget.h"

// ITK
#include "itkCastImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRegionOfInterestImageFilter.h"
//#include "itkVector.h"

// Qt
#include <QFileDialog>
#include <QIcon>
#include <QProgressDialog>
#include <QTextEdit>
#include <QtConcurrentRun>

// VTK
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkFloatArray.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleImage.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <vtkPointData.h>
#include <vtkPointPicker.h>
#include <vtkProperty2D.h>
#include <vtkPolyDataMapper.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkImageSliceMapper.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

// PCL
#include <pcl/features/vfh.h>

// Boost
#include <boost/make_shared.hpp>

// Custom
#include "Helpers.h"
#include "Types.h"
#include "PointSelectionStyle3D.h"
#include "VTKtoPCL.h"
#include "ComputeNormals.h"

void VFHComparisonWidget::on_actionHelp_activated()
{
  QTextEdit* help=new QTextEdit();

  help->setReadOnly(true);
  help->append("<h1>Compare descriptors</h1>\
  Load a point cloud <br/>\
  Ctrl+click to select a point. <br/>\
  Click Compare.<br/>"
  );
  help->show();
}

void VFHComparisonWidget::on_actionQuit_activated()
{
  exit(0);
}

// Constructor
VFHComparisonWidget::VFHComparisonWidget() : MarkerRadius(.05), PCLCloud(new InputCloud), PCLCloudWithNormals(new NormalsCloud)
{
  this->ProgressDialog = new QProgressDialog();
  SharedConstructor();
};

void VFHComparisonWidget::SharedConstructor()
{
  this->setupUi(this);

  this->Mask = MaskImageType::New();

  //this->PCLCloud = boost::make_shared<InputCloud>(*(new InputCloud));
  //this->PCLCloudWithNormals = boost::make_shared<NormalsCloud>(*(new NormalsCloud));
  
  this->PointPicker = vtkSmartPointer<vtkPointPicker>::New();
  this->PointPicker->PickFromListOn();

  // Point cloud
  this->PointCloud = vtkSmartPointer<vtkPolyData>::New();

  this->PointCloudMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  this->PointCloudMapper->SetInputConnection(this->PointCloud->GetProducerPort());

  this->PointCloudActor = vtkSmartPointer<vtkActor>::New();
  this->PointCloudActor->SetMapper(this->PointCloudMapper);

  // Marker
  this->MarkerSource = vtkSmartPointer<vtkSphereSource>::New();
  this->MarkerSource->SetRadius(this->MarkerRadius);
  this->MarkerSource->Update();

  this->MarkerMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  this->MarkerMapper->SetInputConnection(this->MarkerSource->GetOutputPort());

  this->MarkerActor = vtkSmartPointer<vtkActor>::New();
  this->MarkerActor->SetMapper(this->MarkerMapper);

  // Renderer
  this->Renderer = vtkSmartPointer<vtkRenderer>::New();
  this->Renderer->AddActor(this->PointCloudActor);
  this->Renderer->AddActor(this->MarkerActor);

  this->SelectionStyle = PointSelectionStyle3D::New();
  this->SelectionStyle->AddObserver(this->SelectionStyle->SelectedPointEvent, this, &VFHComparisonWidget::SelectedPointCallback);

  // Qt things
  this->qvtkWidget->GetRenderWindow()->AddRenderer(this->Renderer);

  connect(&this->FutureWatcher, SIGNAL(finished()), this->ProgressDialog , SLOT(cancel()));

  // Setup icons
  QIcon openIcon = QIcon::fromTheme("document-open");

  actionOpenPointCloud->setIcon(openIcon);
  this->toolBar_left->addAction(actionOpenPointCloud);
}

void VFHComparisonWidget::SelectedPointCallback(vtkObject* caller, long unsigned int eventId, void* callData)
{
  double p[3];
  this->PointCloud->GetPoint(this->SelectionStyle->SelectedPointId, p);
  this->MarkerActor->SetPosition(p);
}

void VFHComparisonWidget::Refresh()
{
  this->qvtkWidget->GetRenderWindow()->Render();
}

// void VFHComparisonWidget::on_cmbArrayName_activated(int value)
// {
//   this->NameOfArrayToCompare = this->comboBox->currentText().toStdString();
// }

void VFHComparisonWidget::LoadMask(const std::string& fileName)
{
  std::cout << "Reading mask " << fileName << std::endl;
  
  typedef itk::ImageFileReader<MaskImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(fileName);
  reader->Update();

  Helpers::DeepCopy(reader->GetOutput(), this->Mask.GetPointer());
}

void VFHComparisonWidget::LoadPointCloud(const std::string& fileName)
{
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(fileName.c_str());

  reader->Update();

  this->PointCloud->DeepCopy(reader->GetOutput());

  // Start the computation.
//   QFuture<void> future = QtConcurrent::run(reader.GetPointer(), static_cast<void(vtkXMLPolyDataReader::*)()>(&vtkXMLPolyDataReader::Update));
//   this->FutureWatcher.setFuture(future);
//   this->ProgressDialog->setMinimum(0);
//   this->ProgressDialog->setMaximum(0);
//   this->ProgressDialog->setLabelText("Opening file...");
//   this->ProgressDialog->setWindowModality(Qt::WindowModal);
//   this->ProgressDialog->exec();

  this->PointCloudMapper->SetInputConnection(this->PointCloud->GetProducerPort());

  this->PointCloudActor->GetProperty()->SetRepresentationToPoints();

  this->Renderer->ResetCamera();

  this->PointPicker->AddPickList(this->PointCloudActor);

  this->qvtkWidget->GetRenderWindow()->GetInteractor()->SetPicker(this->PointPicker);
  this->SelectionStyle->Points = this->PointCloud;
  this->SelectionStyle->SetCurrentRenderer(this->Renderer);
  this->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(this->SelectionStyle);

  this->Renderer->ResetCamera();

  std::cout << "Converting to PCL..." << std::endl;
  //VTKtoPCL(this->PointCloud.GetPointer(), this->PCLCloud.get());
  VTKtoPCL(this->PointCloud, this->PCLCloud.get());

  std::cout << "Computing normals..." << std::endl;
  this->NormalComputer.Compute(this->PCLCloud, this->PCLCloudWithNormals);
}

void VFHComparisonWidget::on_btnCompute_clicked()
{
  ComputeDifferences();
}

void VFHComparisonWidget::ComputeFeatures()
{
  // Only compute the descriptor on a subset of the points
  // Input requirements: 'polyData' must have a vtkIntArray called "OriginalPixel" that has 2-tuples indicating which pixel in the depth image the point corresponds to

  std::cout << "There are " << this->PointCloud->GetNumberOfPoints() << " vtp points." << std::endl;

  // Initalize 'output'
  //OutputCloud::Ptr featureCloud = boost::make_shared<OutputCloud>(*(new OutputCloud)); // This works, but maybe a bad idea?
  OutputCloud::Ptr featureCloud(new OutputCloud);
  featureCloud->resize(this->PointCloud->GetNumberOfPoints());

  std::cout << "Creating index map..." << std::endl;
  std::map<itk::Index<2>, unsigned int, itk::Index<2>::LexicographicCompare> coordinateMap;

  vtkIntArray* indexArray = vtkIntArray::SafeDownCast(this->PointCloud->GetPointData()->GetArray("OriginalPixel"));
  for(vtkIdType pointId = 0; pointId < this->PointCloud->GetNumberOfPoints(); ++pointId)
    {
    //int* pixelIndexArray;
    int pixelIndexArray[2];
    indexArray->GetTupleValue(pointId, pixelIndexArray);

    itk::Index<2> pixelIndex;
    pixelIndex[0] = pixelIndexArray[0];
    pixelIndex[1] = pixelIndexArray[1];
    coordinateMap[pixelIndex] = pointId;
    }

  unsigned int patch_half_width = 5;

  std::cout << "Computing descriptors..." << std::endl;
  itk::ImageRegion<2> fullRegion = this->Mask->GetLargestPossibleRegion();
  itk::ImageRegionConstIteratorWithIndex<MaskImageType> imageIterator(this->Mask, fullRegion);
  std::cout << "Full region: " << fullRegion << std::endl;

  OutputCloud::PointType emptyPoint;
  for(unsigned int component = 0; component < 308; ++component)
    {
    emptyPoint.histogram[component] = 0.0f;
    }

  //std::fstream fout("/home/doriad/temp/output.txt");
  while(!imageIterator.IsAtEnd())
    {
    //std::cout << "Computing descriptor for pixel " << imageIterator.GetIndex() << std::endl;
    //fout << "Computing descriptor for pixel " << imageIterator.GetIndex() << std::endl;
    itk::ImageRegion<2> patchRegion = Helpers::GetRegionInRadiusAroundPixel(imageIterator.GetIndex(), patch_half_width);
    //std::cout << "patchRegion: " << patchRegion << std::endl;
    if(!fullRegion.IsInside(patchRegion))
      {
      ++imageIterator;
      continue;
      }

    // Get a list of the pointIds in the region
    //std::vector<unsigned int> pointIds;
    std::vector<int> pointIds;
  
    itk::ImageRegionConstIteratorWithIndex<MaskImageType> patchIterator(this->Mask, patchRegion);
    while(!patchIterator.IsAtEnd())
      {
      if(!patchIterator.Get())
        {
        pointIds.push_back(coordinateMap[patchIterator.GetIndex()]);
        }
      ++patchIterator;
      }

    if(pointIds.size() < 2)
      {
      unsigned int currentPointId = coordinateMap[imageIterator.GetIndex()];

      featureCloud->points[currentPointId] = emptyPoint;
      ++imageIterator;
      continue;
      }
    //std::cout << "There are " << pointIds.size() << " points in this patch." << std::endl;
    
//     {
//       itk::Index<2> testIndex = {{227, 93}};
//       if(testIndex == imageIterator.GetIndex())
//         {
//         std::cout << "There are " << pointIds.size() << " points in this patch." << std::endl;
//         for(unsigned int i = 0; i < pointIds.size(); ++i)
//           {
//           std::cout << "Id: " << i << " : " << pointIds[i] << input->points[pointIds[i]] << std::endl;
//           }
//         }
//     }

    // Setup the feature computation
    pcl::VFHEstimation<InputCloud::PointType, pcl::Normal, OutputCloud::PointType> vfhEstimation;

    //vfhEstimation.setIndices(&pointIds);
    vfhEstimation.setIndices(boost::make_shared<std::vector<int> >(pointIds));

    // Provide the original point cloud (without normals)
    vfhEstimation.setInputCloud (this->PCLCloud);

    // Provide the point cloud with normals
    vfhEstimation.setInputNormals(this->PCLCloudWithNormals);

    // vfhEstimation.setInputWithNormals(cloud, cloudWithNormals); VFHEstimation does not have this function
    // Use the same KdTree from the normal estimation
    vfhEstimation.setSearchMethod (this->NormalComputer.Tree);

    //vfhEstimation.setRadiusSearch (0.2);

    // Actually compute the VFH for this subset of points
    OutputCloud::Ptr vfhFeature(new OutputCloud);
    vfhEstimation.compute (*vfhFeature);

    unsigned int currentPointId = coordinateMap[imageIterator.GetIndex()];

    featureCloud->points[currentPointId] = vfhFeature->points[0];

    ++imageIterator;
    }

  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName("Descriptors");
  descriptors->SetNumberOfComponents(308);
  descriptors->SetNumberOfTuples(this->PointCloud->GetNumberOfPoints());

//   // Zero all of the descriptors, we may not have one to assign for every point.
//   std::vector<float> zeroVector(308, 0);
// 
//   for(size_t pointId = 0; pointId < outputCloud->points.size(); ++pointId)
//     {
//     descriptors->SetTupleValue(pointId, zeroVector.data());
//     }

  for(vtkIdType pointId = 0; pointId < this->PointCloud->GetNumberOfPoints(); ++pointId)
    {
    descriptors->SetTupleValue(pointId, featureCloud->points[pointId].histogram);
    }

  this->PointCloud->GetPointData()->AddArray(descriptors);
}

void VFHComparisonWidget::ComputeDifferences()
{
  ComputeFeatures();

  vtkIdType numberOfPoints = this->PointCloud->GetNumberOfPoints();
  std::cout << "There are " << numberOfPoints << " points." << std::endl;

  vtkIdType selectedPointId = this->SelectionStyle->SelectedPointId;
  std::cout << "selectedPointId: " << selectedPointId << std::endl;

  if(selectedPointId < 0 || selectedPointId >= numberOfPoints)
    {
    std::cerr << "You must select a point to compare!" << std::endl;
    return;
    }

  std::string nameOfArrayToCompare = "Descriptors";

  vtkDataArray* descriptorArray = this->PointCloud->GetPointData()->GetArray(nameOfArrayToCompare.c_str());

  if(!descriptorArray)
    {
    throw std::runtime_error("Array not found!"); // The array should always be found because we are selecting it from a list of available arrays!
    }

  std::cout << "There are " << descriptorArray->GetNumberOfComponents() << " components in the descriptor." << std::endl;
  double* selectedDescriptor = new double[descriptorArray->GetNumberOfComponents()]; // This must be double because it is VTK's internal storage type
  descriptorArray->GetTuple(selectedPointId, selectedDescriptor);
  //std::cout << "selectedDescriptor: " << selectedDescriptor[0] << " " << selectedDescriptor[1] << std::endl;

  vtkSmartPointer<vtkFloatArray> differences = vtkSmartPointer<vtkFloatArray>::New();
  std::string descriptorDifferenceName = nameOfArrayToCompare + "_Differences";
  differences->SetName(descriptorDifferenceName.c_str());
  differences->SetNumberOfComponents(1);
  differences->SetNumberOfTuples(numberOfPoints);

  for(vtkIdType pointId = 0; pointId < this->PointCloud->GetNumberOfPoints(); ++pointId)
    {
    double* currentDescriptor = descriptorArray->GetTuple(pointId);
    //std::cout << "descriptor " << pointId << " : " << currentDescriptor[0] << " " << currentDescriptor[1] << std::endl;
    float difference = Helpers::ArrayDifference(selectedDescriptor, currentDescriptor, descriptorArray->GetNumberOfComponents());
    differences->SetValue(pointId, difference);
    }

  this->PointCloud->GetPointData()->AddArray(differences);
  this->PointCloud->GetPointData()->SetActiveScalars(descriptorDifferenceName.c_str());

  float range[2];
  differences->GetValueRange(range);
  vtkSmartPointer<vtkLookupTable> lookupTable = vtkSmartPointer<vtkLookupTable>::New();
  std::cout << "Range: " << range[0] << ", " << range[1] << std::endl;
  lookupTable->SetTableRange(range[0], range[1]);
  //lookupTable->SetHueRange(0, 1); // Don't do this, because 0 and 1 are the same in the H space of HSV!
  lookupTable->SetHueRange(0, .5);

  this->PointCloudMapper->SetLookupTable(lookupTable);
  //std::cout << "UseLookupTableScalarRange " << this->Pane->PointCloudMapper->GetUseLookupTableScalarRange() << std::endl;
  //this->Pane->PointCloudMapper->SetUseLookupTableScalarRange(false);

  // Without this, only a small band of colors is produce around the point.
  // I'm not sure why the scalar range of the data set is not the same?
  this->PointCloudMapper->SetUseLookupTableScalarRange(true);

  this->qvtkWidget->GetRenderWindow()->Render();
  //PopulateArrayNames(this->PointCloud);
}

void VFHComparisonWidget::on_actionSave_activated()
{
  // Get a filename to save
  QString fileName = QFileDialog::getSaveFileName(this, "Save File", ".", "Point Clouds (*.vtp)");

  std::cout << "Got filename: " << fileName.toStdString() << std::endl;
  if(fileName.toStdString().empty())
    {
    std::cout << "Filename was empty." << std::endl;
    return;
    }

  SavePointCloud(fileName.toStdString());
}

void VFHComparisonWidget::SavePointCloud(const std::string& fileName)
{
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(fileName.c_str());
  writer->SetInputConnection(this->PointCloud->GetProducerPort());
  writer->Write();
}

void VFHComparisonWidget::on_actionOpenPointCloud_activated()
{
  // Get a filename to open
  QString fileName = QFileDialog::getOpenFileName(this, "Open File", ".", "Point Clouds (*.vtp)");

  std::cout << "Got filename: " << fileName.toStdString() << std::endl;
  if(fileName.toStdString().empty())
    {
    std::cout << "Filename was empty." << std::endl;
    return;
    }

  LoadPointCloud(fileName.toStdString());
}
