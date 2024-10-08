
from interface import ContractInterface
import os
from web3 import Web3, HTTPProvider
from solcx import *



w3 =  Web3(HTTPProvider('http://127.0.0.1:7545'))


p = os.path.abspath('../contracts')

con = ContractInterface(w3, 'crowdsourcing', p)

con.compile_source_files()

ownerAcc=w3.eth.accounts[3] #person who deploys(owner)
price=0.03 #this the contract value

#owner deploys the contract
contractadress=con.deploy_contract(deployment_params={'from':ownerAcc,'value': w3.to_wei(price,'ether')}) #send the payment value while deploying
con.get_instance()
#lets check the balance
balance=w3.eth.get_balance(contractadress)
print("after deploying balance of the contract: ",w3.from_wei(balance,"ether"))
#worker accepts the contract
acceptAcc=w3.eth.accounts[4] #turtlebot user
collateral=0.02
con.send('accept_contract',tx_params={'from':acceptAcc,'value':w3.to_wei(collateral,'ether'), 'to':contractadress}) #acccepting the contract while paying the collateral

#lets check the balance
balance=w3.eth.get_balance(contractadress)
print("after accepting balance of the contract: ",w3.from_wei(balance,"ether"))

#######---------if cancelling the contract
#con.send('cancel_contract',tx_params={'from':acceptAcc,'to':contractadress})

########--------lets check the balance of the contract after cancelling
#balance=w3.eth.get_balance(contractadress)
#print("after cancelling balance of the contract: ",w3.from_wei(balance,"ether"))

#########-------lets check the balance of acceptor(worker) now

#balance=w3.eth.get_balance(acceptAcc)
#print("after cancelling balance of accept account: ",w3.from_wei(balance,"ether"))

#########-------After completing the contract
con.send('releaseNpay',tx_params={'from':ownerAcc,'to':contractadress})

#########-------lets check the balance of acceptor(worker) now
balance=w3.eth.get_balance(acceptAcc)
print("after completing balance of accept account: ",w3.from_wei(balance,"ether"))

########--------lets check the balance of the contract after completing
balance=w3.eth.get_balance(contractadress)
print("after cancelling balance of the contract: ",w3.from_wei(balance,"ether"))

