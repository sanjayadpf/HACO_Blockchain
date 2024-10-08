// SPDX-License-Identifier: MIT
pragma solidity ^0.5.0;

contract SwarmContract {
    address public owner;
    address payable public worker;  // Declare worker as payable
    uint256 public reward;
    uint256 public collateral;
    bool public taskAccepted;
    bool public taskCompleted;

    event TaskAccepted(address indexed worker, uint256 collateral);
    event TaskCompleted(address indexed worker, uint256 reward);
    event TaskCancelled(address indexed worker, uint256 collateral);

    // Modify constructor to accept reward and collateral
    constructor(uint256 _reward, uint256 _collateral) public payable {
        require(msg.value == _reward + _collateral, "Incorrect total value sent");
        
        owner = msg.sender;
        reward = _reward;
        collateral = _collateral;
        taskAccepted = false;
        taskCompleted = false;
    }

    modifier onlyOwner() {
        require(msg.sender == owner, "Only the owner can call this function.");
        _;
    }

    modifier onlyWorker() {
        require(msg.sender == worker, "Only the worker can call this function.");
        _;
    }

    modifier taskNotAccepted() {
        require(!taskAccepted, "Task has already been accepted.");
        _;
    }

    modifier taskAcceptedModifier() {
        require(taskAccepted, "Task has not been accepted yet.");
        _;
    }

    function acceptTask() external payable taskNotAccepted {
        require(msg.value > 0, "Collateral is required to accept the task.");
        worker = msg.sender;
        collateral = msg.value;
        taskAccepted = true;

        emit TaskAccepted(worker, collateral);
    }

    function completeTask() external onlyWorker taskAcceptedModifier {
        require(!taskCompleted, "Task has already been completed.");
        taskCompleted = true;

        // Transfer the reward and collateral to the worker
        worker.transfer(reward + collateral);

        emit TaskCompleted(worker, reward);
    }

    function cancelTask() external onlyOwner taskAcceptedModifier {
        require(!taskCompleted, "Task has already been completed.");
        
        // Refund the collateral to the worker
        worker.transfer(collateral);
        
        // Mark task as not accepted
        taskAccepted = false;

        emit TaskCancelled(worker, collateral);
    }
}
