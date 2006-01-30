#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCommand.h"
#include "itkSimpleFilterWatcher.h"
#include "itkChangeInformationImageFilter.h"

#include "itkIsotropicResolutionImageFilter.h"


int main(int, char * argv[])
{
  const int dim = 2;
  
  typedef unsigned char PType;
  typedef itk::Image< PType, dim > IType;

  typedef itk::ImageFileReader< IType > ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( argv[3] );

  typedef itk::ChangeInformationImageFilter< IType > ChangeInfoType;
  ChangeInfoType::Pointer changeInfo = ChangeInfoType::New();
  itk::Vector<double, 2> spacing;
  spacing[0] = 0.1;
  spacing[1] = 0.33;
  changeInfo->SetInput(reader->GetOutput());
  changeInfo->SetOutputSpacing(spacing);
  changeInfo->SetChangeSpacing(true);

  typedef itk::IsotropicResolutionImageFilter< IType, IType > FilterType;
  FilterType::Pointer filter = FilterType::New();
  filter->SetInput( changeInfo->GetOutput() );
  filter->SetMaximumIncrease( atof(argv[1]) );
  filter->SetNearestNeighbor( atoi(argv[2]) );

  itk::SimpleFilterWatcher watcher(filter, "filter");

  typedef itk::ImageFileWriter< IType > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( filter->GetOutput() );
  writer->SetFileName( argv[4] );
  writer->Update();

  return 0;
}

