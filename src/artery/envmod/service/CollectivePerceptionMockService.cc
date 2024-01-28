/*
* Artery V2X Simulation Framework
* Copyright 2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/envmod/service/CollectivePerceptionMockService.h"

#include "artery/envmod/sensor/FovSensor.h"
#include "artery/envmod/service/CollectivePerceptionMockMessage.h"
#include "artery/utility/InitStages.h"
#include "artery/utility/PointerCheck.h"

#include <omnetpp/checkandcast.h>
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>

#include <fstream>

namespace artery
{

Define_Module(CollectivePerceptionMockService)

namespace
{

omnetpp::simsignal_t camSentSignal = omnetpp::cComponent::registerSignal("CamSent");
omnetpp::simsignal_t cpmSentSignal = omnetpp::cComponent::registerSignal("CpmSent");
omnetpp::simsignal_t cpmReceivedSignal = omnetpp::cComponent::registerSignal("CpmReceived");

} // namespace


CollectivePerceptionMockService::~CollectivePerceptionMockService()
{
    std::ofstream generateFile;
    std::ofstream receiveFile;
    std::ofstream indicateFile;

    generateFile.open ("results/generate.txt", std::ios_base::app);
    receiveFile.open ("results/receive.txt", std::ios_base::app);
    indicateFile.open ("results/indicate.txt", std::ios_base::app);

    //    generatedCPMs << "]";
    //    receivedCPMs << "]";


    generateFile << generatedCPMs.rdbuf();
    generateFile.close();

    receiveFile << receivedCPMs.rdbuf();
    receiveFile.close();


    indicateFile << indicatedCPMs.rdbuf();
    indicateFile.close();

    //    generatedCPMs.clear();
    //    receivedCPMs.clear();

    cancelAndDelete(mTrigger);
}

int CollectivePerceptionMockService::numInitStages() const
{
    return InitStages::Total;
}

void CollectivePerceptionMockService::initialize(int stage)
{
    if (stage == InitStages::Prepare) {
        ItsG5Service::initialize();
        mPositionProvider = &getFacilities().get_const<PositionProvider>();
        mEnvironmentModel = &getFacilities().get_const<LocalEnvironmentModel>();

        mDccProfile = par("dccProfile");
        mLengthHeader = par("lengthHeader");
        mLengthFovContainer = par("lengthFovContainer");
        mLengthObjectContainer = par("lengthObjectContainer");

        mGenerateAfterCam = par("generateAfterCam");
        mCpmOffset = par("cpmOffset");
        mCpmInterval = par("cpmInterval");
    } else if (stage == InitStages::Self) {
        mTrigger = new omnetpp::cMessage("triggger mock-up CPM");
        if (mGenerateAfterCam) {
            subscribe(camSentSignal);
        }
        mHostId = getFacilities().get_const<Identity>().host->getId();
    } else if (stage == InitStages::Propagate) {
        for (const Sensor* sensor : mEnvironmentModel->getSensors()) {
            // consider only sensors with a field of view
            if (auto fovSensor = dynamic_cast<const FovSensor*>(sensor)) {
                CollectivePerceptionMockMessage::FovContainer fovContainer;
                fovContainer.sensorId = sensor->getId();
                fovContainer.position = sensor->position();
                fovContainer.fov = fovSensor->getFieldOfView();
                mFovContainers.emplace_back(std::move(fovContainer));
                mSensors.insert(sensor);
            }
        }
    }
}

void CollectivePerceptionMockService::handleMessage(omnetpp::cMessage* msg)
{
    if (msg == mTrigger) {
        generatePacket();
    } else {
        receivedCPMs << "Received : " <<  msg->getDisplayString() << "\n";
        ItsG5Service::handleMessage(msg);
    }
}

void CollectivePerceptionMockService::receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t signal,
        omnetpp::cObject* obj, omnetpp::cObject*)
{
    if (signal == camSentSignal) {
        scheduleAt(omnetpp::simTime() + mCpmOffset, mTrigger);
    }
}

void CollectivePerceptionMockService::trigger()
{
    if (!mGenerateAfterCam && !mTrigger->isScheduled()) {
        auto channel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CP);
        auto netifc = notNullPtr(getFacilities().get_const<NetworkInterfaceTable>().select(channel));
        vanetza::dcc::TransmitRateThrottle* trc = notNullPtr(netifc->getDccEntity().getTransmitRateThrottle());
        vanetza::dcc::TransmissionLite tx { static_cast<vanetza::dcc::Profile>(mDccProfile), 0 };
        const omnetpp::SimTime interval = std::chrono::duration<double>(trc->interval(tx)).count();
        const omnetpp::SimTime next = omnetpp::simTime() + mCpmOffset;
        if (mTrigger->getArrivalTime() + std::max(interval, mCpmInterval) <= next) {
            scheduleAt(next, mTrigger);
        }
    }
}

void CollectivePerceptionMockService::indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket* packet)
{
    auto cpm = omnetpp::check_and_cast<CollectivePerceptionMockMessage*>(packet);
    emit(cpmReceivedSignal, cpm);
    delete packet;
}

void CollectivePerceptionMockService::generatePacket()
{
    using namespace vanetza;
    btp::DataRequestB req;
    req.destination_port = host_cast<PortNumber>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(mDccProfile);
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto packet = new CollectivePerceptionMockMessage();
    packet->setByteLength(mLengthHeader);

    generatedCPMs << "{"
           << "\"destination_port\":" <<  req.destination_port << ","
           << "\"transport_type\":" << static_cast<int>(geonet::TransportType::SHB) << ","
           << "\"tc_id\":" <<  mDccProfile << ","
           << "\"communication_profile\":" << static_cast<int>(req.gn.communication_profile) << ",";


    if (mFovLast + mFovInterval <= omnetpp::simTime()) {
        packet->setFovContainers(mFovContainers);
        packet->addByteLength(mLengthFovContainer * mFovContainers.size());
        generatedCPMs << "\"fov_containers\": [ ";
        for (CollectivePerceptionMockMessage::FovContainer fovContainer : mFovContainers) {
            generatedCPMs << "{"
                   << "\"angle\":"<< fovContainer.fov.angle.value() << ","
                   << "\"range\":"<< fovContainer.fov.range.value() << ","
                   << "\"position\":"<< static_cast<int>(fovContainer.position) << ","
                   << "\"sensorId\":"<< fovContainer.sensorId
                   << "},";
        }
        generatedCPMs << "],";

        mFovLast = omnetpp::simTime();
        generatedCPMs <<  "\"last_fov\":" << mFovLast << ",";
    }

    std::vector<CollectivePerceptionMockMessage::ObjectContainer> objectContainers;
    using TrackedObject = LocalEnvironmentModel::TrackedObject;
    generatedCPMs << "\"tracked_objects\": [ ";
    for (const TrackedObject& object : mEnvironmentModel->allObjects()) {
        const LocalEnvironmentModel::Tracking& tracking = object.second;
        if (tracking.expired()) {
            // skip objects with lost tracking
            continue;
        }

        for (const auto& sensor : tracking.sensors()) {
            if (mSensors.find(sensor.first) != mSensors.end()) {
                generatedCPMs << "{";
                CollectivePerceptionMockMessage::ObjectContainer objectContainer;
                objectContainer.object = object.first;
                objectContainer.objectId = tracking.id();
                objectContainer.sensorId = sensor.first->getId();
                objectContainer.timeOfMeasurement = sensor.second.last();

                generatedCPMs << "\"objectId\":"<< objectContainer.objectId << ","
                       << "\"sensorId\":"<< objectContainer.sensorId << ","
                       << "\"timeOfMeasurement\":"<< objectContainer.timeOfMeasurement << ",";

                objectContainers.emplace_back(std::move(objectContainer));
                EnvironmentModelObject* o = object.first.lock().get();
                if(o!=NULL) {
                    generatedCPMs << "\"object\": { "
                           << "\"center_x\":" << o->getCentrePoint().x.value() << ","
                           << "\"center_y\":" << o->getCentrePoint().y.value() << ","
                           << "\"external_id\":\"" << o->getExternalId() << "\","
                           << "\"length\":" << o->getLength().value()
                           << ","
                           // TODO: o->getOutline()
                           << "\"radius\":" << o->getRadius().value() << ","
                           << "\"width\":" << o->getWidth().value() << ","
                           << "\"vehicle\": {"
                           << "\"x\":" << o->getVehicleData().position().x.value() << ","
                           << "\"y\":" << o->getVehicleData().position().y.value() << ","
                           << "\"acceleration\":" << o->getVehicleData().acceleration().value() << ","
                           << "\"curvature\":" << o->getVehicleData().curvature().value() << ","
                           << "\"curvature_confidence\":" << o->getVehicleData().curvature_confidence() << ","
                           << "\"station_id\":" << o->getVehicleData().station_id() << ","
                           << "\"station_type\":" << static_cast<int>(o->getVehicleData().getStationType()) << ","
                           << "\"heading\":" << o->getVehicleData().heading().value() << ","
                           << "\"latitude\":" << o->getVehicleData().latitude().value() << ","
                           << "\"longitude\":" << o->getVehicleData().longitude().value() << ","
                           << "\"speed\":" << o->getVehicleData().speed().value() << ","
                           << "\"updated\":" << o->getVehicleData().updated().dbl() << ","
                           << "\"yaw_rate\":" << o->getVehicleData().yaw_rate().value() << ","
                           << "},"
                           << "}";
                }
                generatedCPMs << "},";
            }
        }
    }
    generatedCPMs << "],";

    packet->addByteLength(mLengthObjectContainer * objectContainers.size());
    packet->setObjectContainers(std::move(objectContainers));
    packet->setSourceStation(mHostId);

    generatedCPMs <<  "\"length\":" << packet->getByteLength() << ","
           <<  "\"mHostId\":" << mHostId
        << "}," <<   "\n";



    emit(cpmSentSignal, packet);
    request(req, packet);
}
CollectivePerceptionMockService::CollectivePerceptionMockService():mGenerateAfterCam(false)
{
//    generatedCPMs << "[";
//    receivedCPMs << "[";
}

} // namespace artery
//sed  -E  's/\},]/}]/g' generate.txt > generate_edited.json && sed  -i -E 's/,\}/}/g'  generate_edited.json