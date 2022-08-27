#ifndef EMA_FILTER_HPP
#define EMA_FILTER_HPP

#include <cmath>

class EMAFilter
{
public:
  EMAFilter();
  EMAFilter(int filter_size);
  EMAFilter(double alpha);
  ~EMAFilter();

  double filterData(double data);
  void setStartVal(double data);

  int getFilterSize();
  void setFilterSize(int filter_size);

  double getFilterAlpha();
  void setFilterAlpha(double alpha);

private:
  int _num_observations;
  int _filter_size;
  double _alpha;

  double _previous_data_filtered;
  double _raw_data;
  double _filtered_data;

};

EMAFilter::EMAFilter()
{
  this->_previous_data_filtered = 0.0;
  this->_num_observations = 0;
}

EMAFilter::EMAFilter(int filter_size) : _filter_size(filter_size)
{
  this->_alpha = 1.0 - 1.0/filter_size;
  this->_previous_data_filtered = 0.0;
  this->_num_observations = 0;
}

EMAFilter::EMAFilter(double alpha) : _alpha(alpha)
{
  this->_filter_size = (int) (1.0/(1.0-alpha));
  this->_previous_data_filtered = 0.0;
  this->_num_observations = 0;
}

EMAFilter::~EMAFilter()
{
}

void EMAFilter::setStartVal(double data)
{
  this->_previous_data_filtered = data;
  this->_raw_data = data;
  this->_filtered_data = data;
  this->_num_observations = 0;
}

int EMAFilter::getFilterSize()
{
  return this->_filter_size;
}

void EMAFilter::setFilterSize(int filter_size)
{
  this->_filter_size = filter_size;
  this->_alpha = 1.0 - 1.0/filter_size;
}

double EMAFilter::getFilterAlpha()
{
  return this->_alpha;
}

void EMAFilter::setFilterAlpha(double alpha)
{
  this->_alpha = alpha;
  this->_filter_size = (int) (1.0/(1.0-alpha));
}

double EMAFilter::filterData(double data)
{
  this->_num_observations++;

  this->_raw_data = data;
  this->_filtered_data = _alpha*_previous_data_filtered + (1.0-_alpha)*_raw_data;
  this->_previous_data_filtered = _filtered_data;

  if (_num_observations < _filter_size) this->_filtered_data /= (1.0 - pow(_alpha, _num_observations));

  return this->_filtered_data;
}

#endif // EMA_FILTER_HPP
